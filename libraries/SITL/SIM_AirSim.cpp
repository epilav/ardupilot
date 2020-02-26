/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/* 
	Simulator Connector for AirSim
*/

#include "SIM_AirSim.h"

#include <stdio.h>
#include <arpa/inet.h>
#include <errno.h>

#include <algorithm>
#include <string>
#include <cstring>
#include <cassert>
#include <vector>

#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_HAL/utility/replace.h>

extern const AP_HAL::HAL& hal;

using namespace SITL;

class ArduCopterPacketBuffer final {
public:
    ArduCopterPacketBuffer(const uint8_t* beg, const uint8_t* end) : _beg(beg), _end(end), _pos(beg) {}

    void setIndex(std::size_t pos)
    {
        _pos = _beg + pos;
    }

    void skip(std::size_t offset)
    {
        _pos += offset;
    }

    template <typename T>
    T get()
    {
        assert(_pos + sizeof(T) <= _end);
        T r = *(reinterpret_cast<const T*>(_pos));
        _pos += sizeof(T);
        return r;
    }

    std::string getString()
    {
        auto s = _pos;
        _pos += get<uint32_t>();
        assert(_pos  <= _end);
        return std::string(s, _pos);
    }

    template <typename T>
    std::vector<T> getVector()
    {
        auto size = get<uint32_t>();
        T* start = reinterpret_cast<const T*>(_pos);
        _pos += sizeof(T) * size;
        return std::vector<T>(start, _pos);
    }

private:
    const uint8_t *_pos;
    const uint8_t *_beg;
    const uint8_t *_end;
};

template <>
Vector3f ArduCopterPacketBuffer::get<Vector3f>()
{
    assert(_pos + (sizeof(float) * 3) <= _end);
    auto p = reinterpret_cast<const float*>(_pos);
    _pos += sizeof(float) * 3;
    return Vector3f(p[0], p[1], p[2]);
}


AirSim::AirSim(const char *frame_str) :
	Aircraft(frame_str),
	sock(true)
{
    if (strstr(frame_str, "-copter")) {
        output_type = OutputType::Copter;
    } else if (strstr(frame_str, "-rover")) {
        output_type = OutputType::Rover;
    } else {
        // default to copter
        output_type = OutputType::Copter;
    }

	printf("Starting SITL Airsim type %u\n", (unsigned)output_type);
}

/*
	Create & set in/out socket
*/
void AirSim::set_interface_ports(const char* address, const int port_in, const int port_out)
{
	if (!sock.bind("0.0.0.0", port_in)) {
		printf("Unable to bind Airsim sensor_in socket at port %u - Error: %s\n",
				 port_in, strerror(errno));
		return;
	}
	printf("Bind SITL sensor input at %s:%u\n", "127.0.0.1", port_in);
	sock.set_blocking(false);
	sock.reuseaddress();

	airsim_ip = address;
	airsim_control_port = port_out;
	airsim_sensor_port = port_in;

	printf("AirSim control interface set to %s:%u\n", airsim_ip, airsim_control_port);
}

/*
	Decode and send servos
*/
void AirSim::output_copter(const struct sitl_input &input)
{
    servo_packet pkt;

	for (uint8_t i=0; i<kArduCopterRotorControlCount; i++) {
		pkt.pwm[i] = input.servos[i];
	}

	ssize_t send_ret = sock.sendto(&pkt, sizeof(pkt), airsim_ip, airsim_control_port);
	if (send_ret != sizeof(pkt)) {
		if (send_ret <= 0) {
			printf("Unable to send servo output to %s:%u - Error: %s, Return value: %ld\n",
                   airsim_ip, airsim_control_port, strerror(errno), (long)send_ret);
		} else {
			printf("Sent %ld bytes instead of %lu bytes\n", (long)send_ret, (unsigned long)sizeof(pkt));
		}
	}
}

void AirSim::output_rover(const struct sitl_input &input)
{
    rover_packet pkt;

    pkt.steering = 2*((input.servos[0]-1000)/1000.0f - 0.5f);
    pkt.throttle = 2*((input.servos[2]-1000)/1000.0f - 0.5f);

    ssize_t send_ret = sock.sendto(&pkt, sizeof(pkt), airsim_ip, airsim_control_port);
    if (send_ret != sizeof(pkt)) {
        if (send_ret <= 0) {
            printf("Unable to send control output to %s:%u - Error: %s, Return value: %ld\n",
                     airsim_ip, airsim_control_port, strerror(errno), (long)send_ret);
        } else {
            printf("Sent %ld bytes instead of %lu bytes\n", (long)send_ret, (unsigned long)sizeof(pkt));
        }
    }
}

/*
	Receive new sensor data from simulator
	This is a blocking function
*/
void AirSim::recv_fdm()
{
    // Receive sensor packet
    ssize_t ret = sock.recv(&sensor_buffer[sensor_buffer_len], sizeof(sensor_buffer)-sensor_buffer_len, 100);
    while (ret <= 0) {
        ret = sock.recv(&sensor_buffer[sensor_buffer_len], sizeof(sensor_buffer)-sensor_buffer_len, 100);
    }

    ArduCopterPacketBuffer buffer(sensor_buffer, sensor_buffer + ret);
    
    // Read the TOC
    uint16_t toc_rc         = buffer.get<uint16_t>();
    uint16_t toc_lidar      = buffer.get<uint16_t>();
    uint16_t toc_distance   = buffer.get<uint16_t>();
    uint16_t toc_imu        = buffer.get<uint16_t>();
    uint16_t toc_gps        = buffer.get<uint16_t>();

    // IMU
    // ---
    //
    // {
    //     angular-velocity:        3 floats
    //     linear-acceleration:     3 floats
    //     roll:                    float
    //     pitch:                   float
    //     yaw:                     float
    // }
    if (toc_imu) {
        buffer.setIndex(toc_imu);
        gyro = buffer.get<Vector3f>();
        accel_body = buffer.get<Vector3f>();     
        float x = buffer.get<float>();
        float y = buffer.get<float>();
        float z = buffer.get<float>();
        dcm.from_euler(x, y, z);
    }

    // GPS
    // ---
    //
    // {
    //     latitude:                double
    //     longitude:               double
    //     altitude:                double
    //     velocity:                3 floats
    // }
    if (toc_gps) {
        buffer.setIndex(toc_gps);
        location.lat = buffer.get<double>() * 1.0e7;
        location.lng = buffer.get<double>() * 1.0e7;
        location.alt = buffer.get<double>() * 100.0f;
        velocity_ef = buffer.get<Vector3f>();
    }

    // Lidars
    // ------
    //
    // lidar-count: 8-bit value
    // lidars: variable length array of 
    // {
    //     name-length:             16-bit value
    //     name:                    variable length 8-bit value array
    //     point-cloud-size:        32-bit value
    //     point-cloud:             variable length float array
    // }
    scanner.points.length = 0;
    if (toc_lidar) {
        // TODO(epilav): (re)introduce lidars
    }

    // Distance sensors
    // ----------------
    //
    // distance-conut: 8-bit value
    // distance-sensors: variable length array of 
    // {
    //     name-length:             16-bit value
    //     name:                    variable length 8-bit value array
    //     distance:                float
    //     max-distance:            float
    //     min-distance:            float
    //     roll:                    float
    //     pitch:                   float
    //     yaw:                     float
    // }
    //
    // WARNING!
    // Note that we're limiting the total number of distance sensors to 6. The assumption here is that
    // we have one sensor per side, I suppose. This will work for our purposes, but this is clearly not
    // a good generic solution.
    if (toc_distance) {
        buffer.setIndex(toc_distance);
        uint8_t i = 0, n = std::min(buffer.get<uint8_t>(), static_cast<uint8_t>(6));
        for (; i < n; i++) {
            buffer.skip(buffer.get<uint16_t>());    // Currently we don't care about the name
            distance_sensors[i].valid = true;
            distance_sensors[i].current = buffer.get<float>();
            distance_sensors[i].maximum = buffer.get<float>();
            distance_sensors[i].minimum = buffer.get<float>();
            distance_sensors[i].orientation = buffer.get<Vector3f>();
        }
        for (; i < 6; i++) {
            distance_sensors[i].valid = false;
        }
    }

    // RC
    // --
    //
    // {
    //     channel-count:           8-bit value
    //     channel-values:          variable length float array
    // }
    if (toc_rc) {
        buffer.setIndex(toc_rc);
        uint8_t i = 0, n = std::min(buffer.get<uint8_t>(), static_cast<uint8_t>(8));
        for (; i < n; i++) {
            rcin[i] = buffer.get<float>();
        }
    }
}

/*
  update the AirSim simulation by one time step
*/
void AirSim::update(const struct sitl_input &input)
{
    switch (output_type) {
        case OutputType::Copter:
            output_copter(input);
            break;

        case OutputType::Rover:
            output_rover(input);
            break;
    }

    recv_fdm();

    // update magnetic field
    update_mag_field_bf();
}