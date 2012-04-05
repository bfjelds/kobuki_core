/**
 * @file /cruizcore/include/cruizcore/cruizcore.hpp
 *
 * @brief Cpp interface for a cruizcore gyro device driver.
 *
 * @date 20/08/2010
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef YCS_ROBOTS_ICLEBO_HPP_
#define YCS_ROBOTS_ICLEBO_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/
#include <iostream>
#include <string>
#include <algorithm>
#include <iclebo_comms/iClebo.h>
#include <device_comms/JointState.h>
#include <ecl/threads.hpp>
#include <ecl/devices.hpp>
#include <ecl/time.hpp>

#include <ecl/exceptions/standard_exception.hpp>
#include <packet_handler/packet_finder.hpp>
//#include <iclebo_mainboard/iclebo_mainboard.hpp>
#include "parameters.hpp"

// [ version 1 ]
#include "iclebo_data.hpp"
// [ version 2 ]
#include <iclebo_comms/iCleboHeader.h>
#include "iclebo_default.hpp"
#include "iclebo_ir.hpp"
#include "iclebo_dock_ir.hpp"
#include "iclebo_inertia.hpp"
#include "iclebo_cliff.hpp"
#include "iclebo_current.hpp"
#include "iclebo_magnet.hpp"
#include "iclebo_time.hpp"
#include "iclebo_hw.hpp"
#include "iclebo_fw.hpp"
#include "iclebo_st_gyro.hpp"
#include "iclebo_eeprom.hpp"
#include "iclebo_gp_input.hpp"

#include "iclebo_command.hpp"


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace iclebo {

/*****************************************************************************
** Using
*****************************************************************************/
using ecl::Threadable;
using ecl::Serial;
using ecl::StopWatch;
using ecl::TimeStamp;

union union_sint16 {
	short word;
	unsigned char byte[2];
};

class iCleboPacketFinder : public packet_handler::packetFinder
{
public:
	bool checkSum()
	{
		unsigned int packet_size( buffer.size() );
		unsigned char cs(0);
		for( unsigned int i=2; i<packet_size; i++ )
		{
			cs ^= buffer[i];
		}

		return cs ? false : true;
	}

};


/*****************************************************************************
** Interface [CruizCore]
*****************************************************************************/
/**
 * @brief  The device driver for a cruizcore gyro.
 *
 * The actual device driver for the cruizcore gyro. This driver simply provides
 * an api from which a program can be written to interact with the device.
 * It does not actually provide the control loop per se.
 *
 * <b>Signals</b>
 *
 * This class accepts a string via the parameter interface which is used as the
 * base namespace for all sigslots that can be connected. These sigslots
 * enable runtime handling and/or debugging of various features.
 * The string id's associated with these connections
 * are listed below where 'ns' is the namespace as mentioned.
 *
 * - 'ns'/raw_data_received : emits a bytearray when a goo packet is received.
 * - 'ns'/raw_data_sent : emits a bytearray when sending a goo command packet.
 * - 'ns'/serial_timeout : simple emit to notify if an expected packet timed out.
 * - 'ns'/invalid_packet : emits a bytearray whenever a mangled packet is received.
 **/
class iClebo : public Threadable {
public:
	iClebo() : is_connected(false), is_running(false), is_enabled(false), tick_to_mm(0.0845813406577f), tick_to_rad(0.00201384144460884f){}
	//iClebo(Parameters &parameters) throw(ecl::StandardException);
	~iClebo(){ serial.close(); is_connected=false; is_running=false; is_enabled=false; }

	/*********************
	** Configuration
	**********************/
	void runnable();
	void init(Parameters &parameters) throw(ecl::StandardException);
	bool connected() const { return is_connected; }
	bool isEnabled() const { return is_enabled; }
	void reset();
	//const device_comms::ns::Gyro& data() const { return gyro_data; }	
	bool run();
	bool stop();
	void close();
	void getData(iclebo_comms::iClebo&);
	void getData2(iclebo_comms::iClebo&);
	void getDefaultData(iclebo_comms::iClebo&);
	void getIRData(iclebo_comms::iCleboIR&);
	void getDockIRData(iclebo_comms::iCleboDockIR&);
	void getInertiaData(iclebo_comms::iCleboInertia&);
	void getCliffData(iclebo_comms::iCleboCliff&);
	void getCurrentData(iclebo_comms::iCleboCurrent&);
	void getMagnetData(iclebo_comms::iCleboMagnet&);
	void getHWData(iclebo_comms::iCleboHW&);
	void getFWData(iclebo_comms::iCleboFW&);
	void getTimeData(iclebo_comms::iCleboTime&);
	void getStGyroData(iclebo_comms::iCleboStGyro&);
	void getEEPROMData(iclebo_comms::iCleboEEPROM&);
	void getGpInputData(iclebo_comms::iCleboGpInput&);

	void getJointState(device_comms::JointState&);
	void setCommand(double, double);
	void sendCommand();
	void sendCommand( const iclebo_comms::iCleboCommandConstPtr &data );
	void pubtime(const char *);

private:
	//iCleboMainboardDriver iclebo_receiver;
	StopWatch stopwatch;

	unsigned short last_timestamp;
	double last_velocity_left, last_velocity_right;
	double last_diff_time;

	unsigned short last_tick_left, last_tick_right;
	double last_rad_left, last_rad_right;
	double last_mm_left, last_mm_right;

	short v,w;
	short radius;
	short speed;
	double bias;	//wheelbase, wheel_to_wheel, in [m]

	std::string device_id;
	std::string device_type;
	std::string protocol_version;
	bool is_connected;	// True if there's a serial/usb connection open.
	//PacketHandler packet_handler;
	//Packet::Buffer buffer;
	//device_comms::ns::Gyro gyro_data;
	bool is_running;
	bool is_enabled;

	unsigned int count;
	const double tick_to_mm, tick_to_rad;

	Serial serial;
	iCleboData data;
	iCleboData2 data2;

	// [ vserion 2 ]
	iCleboDefaultData iclebo_default;
	iCleboIRData iclebo_ir;
	iCleboDockIRData iclebo_dock_ir;
	iCleboInertiaData iclebo_inertia;
	iCleboCliffData iclebo_cliff;
	iCleboCurrentData iclebo_current;
	iCleboMagnetData iclebo_magnet;
	iCleboTimeData iclebo_time;
	iCleboHWData iclebo_hw;
	iCleboFWData iclebo_fw;
	iCleboStGyroData iclebo_st_gyro;
	iCleboEEPROMData iclebo_eeprom;
	iCleboGpInputData iclebo_gp_input;

	iCleboCommandData iclebo_command;

	iCleboPacketFinder packet_finder;
	iCleboPacketFinder::BufferType data_buffer;
	ecl::PushAndPop<unsigned char> command_buffer;

	ecl::Signal<> sig_wheel_state, sig_sensor_data;
	ecl::Signal<> 
		sig_default ,
		sig_ir      ,
		sig_dock_ir ,
		sig_inertia ,
		sig_cliff   ,
		sig_current ,
		sig_magnet  ,
		sig_hw      ,
		sig_fw      ,
		sig_time    ,
		sig_st_gyro ,
		sig_eeprom  ,
		sig_gp_input;
//		sig_reserved0;
	std::set<unsigned char> sig_index;
};

} // namespace cruizcore


#endif /* YCS_ROBOTS_ICLEBO_HPP_ */