import os
import dynamixel_sdk as dxl  # Uses Dynamixel SDK library
import time
import numpy as np

class hw_interface(object):
    """docstring for hw_interface"""
    def __init__(self, args):
        super(hw_interface, self).__init__()
        self.args = args
        self.motorStep_to_radians = args.motorStep_to_radians
        self.velocity_limit = args.velocity_limit
        # Create PortHandler instance
        self.portHandler = dxl.PortHandler(args.DEVICENAME)
        # self.portHandler.setPacketTimeout(3.0)

        # Create PacketHandler instance
        self.packetHandler = dxl.PacketHandler(args.PROTOCOL_VERSION)

        # Open port
        if not self.portHandler.openPort():
            print("Failed to open the port")
            quit()

        # Set the baudrate
        if not self.portHandler.setBaudRate(args.BAUDRATE):
            print("Failed to change the baudrate")
            quit()

        self.init_motors()
        # Create GroupSyncWrite instance for goal velocity
        self.groupSyncWrite = dxl.GroupSyncWrite(self.portHandler, self.packetHandler, args.ADDR_PRO_GOAL_VELOCITY, args.LEN_PRO_GOAL_VELOCITY)


        # Create GroupSyncRead instance for present position
        self.groupSyncRead = dxl.GroupSyncRead(self.portHandler, self.packetHandler, args.ADDR_PRESENT_POSITION, args.LEN_PRO_PRESENT_POSITION)

        # Add each motor ID to the GroupSyncRead instance
        for motor_id in args.motor_ids:
            addparam_result = self.groupSyncRead.addParam(motor_id)
            if not addparam_result:
                print(f"Failed to add motor ID {motor_id} to GroupSyncRead")
                quit()
        self.param_goal_velocity = "initial_value"


    def init_motors(self):
        index = 0
        for motor_id in self.args.motor_ids:

            # Disable torque
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, motor_id, self.args.ADDR_TORQUE_ENABLE, 0)

            # Change operating mode to position control
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, motor_id, self.args.ADDR_OPERATING_MODE, self.args.POSITION_CONTROL_MODE)
            
            if dxl_comm_result != 0:
                print(f"Failed to set Position Control mode for motor {motor_id}: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
                return
            elif dxl_error != 0:
                print(f"Dynamixel error in Position Control mode setup for motor {motor_id}: {self.packetHandler.getRxPacketError(dxl_error)}")

            # Enable torque
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, motor_id, self.args.ADDR_TORQUE_ENABLE, 1)

            # Set goal position to
            dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, motor_id, self.args.ADDR_GOAL_POSITION, self.args.motor_offsets[index])# + int(self.args.motor_steps/4))
            if dxl_comm_result != 0:
                print(f"Failed to send motor {motor_id} to position 0: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
                return
            elif dxl_error != 0:
                print(f"Dynamixel error in setting Goal Position for motor {motor_id}: {self.packetHandler.getRxPacketError(dxl_error)}")

            # Allow time for the motor to reach position 0
            time.sleep(1.5)  # Adjust as needed based on motor speed and position range

            # Disable torque
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, motor_id, self.args.ADDR_TORQUE_ENABLE, 0)

            # Change operating mode to velocity control
            # dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, motor_id, self.args.ADDR_OPERATING_MODE, self.args.VELOCITY_CONTROL_MODE)
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, motor_id, self.args.ADDR_OPERATING_MODE, self.args.TORQUE_CONTROL_MODE)
            if dxl_comm_result != 0:
                print(f"Failed to set Velocity Control mode for motor {motor_id}: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
            elif dxl_error != 0:
                print(f"Dynamixel error in Velocity Control mode setup for motor {motor_id}: {self.packetHandler.getRxPacketError(dxl_error)}")
            else:
                print(f"Motor {motor_id} initialized: set to Velocity Control Mode.")

            # Enable torque
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, motor_id, self.args.ADDR_TORQUE_ENABLE, 1)

            index +=1


    def enable_torque(self):
        TORQUE_ENABLE = 1
        for motor_id in self.args.motor_ids:
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, motor_id, self.args.ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
            if dxl_comm_result != dxl.COMM_SUCCESS:
                print(f"Failed to enable torque for motor {motor_id}: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
            elif dxl_error != 0:
                print(f"Error enabling torque for motor {motor_id}: {self.packetHandler.getRxPacketError(dxl_error)}")
            else:
                print(f"Enabled torque for motor {motor_id}")

    def disable_torque(self):
        TORQUE_DISABLE = 0
        for motor_id in self.args.motor_ids:
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, motor_id, self.args.ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
            if dxl_comm_result != dxl.COMM_SUCCESS:
                print(f"Failed to disable torque for motor {motor_id}: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
            elif dxl_error != 0:
                print(f"Error disabling torque for motor {motor_id}: {self.packetHandler.getRxPacketError(dxl_error)}")
            else:
                print(f"Motor {motor_id} is disabled!!")



    def set_velocity_sync(self, goal_velocities):
        # Add goal velocities for each motor to the GroupSyncWrite packet
        for motor_id, goal_velocity in zip(self.args.motor_ids, goal_velocities):
            #print("Type of l:", type(goal_velocity))
            if (goal_velocity/ self.motorStep_to_radians)>=self.velocity_limit or (goal_velocity/ self.motorStep_to_radians)<=-self.velocity_limit:
                print('inside if id', motor_id,'goal_velocity is: ', goal_velocity/ self.motorStep_to_radians)
                print('inside if radians', motor_id,'goal_velocity is: ', goal_velocity)
                #raise Exception("p1")
            else:
                print('outside if id', motor_id, 'goal_velocity is:', goal_velocity)
            # print('goal_velocity', goal_velocity)
            goal_velocity = max(min(int(goal_velocity / self.motorStep_to_radians), self.velocity_limit), -self.velocity_limit)# max(-a, min(a, value))

            
            #print("Type of l:", type(goal_velocity))
            # Convert goal velocity to byte array (Dynamixel uses little-endian)
            self.param_goal_velocity = [
                dxl.DXL_LOBYTE(dxl.DXL_LOWORD(goal_velocity)),
                dxl.DXL_HIBYTE(dxl.DXL_LOWORD(goal_velocity)),
                dxl.DXL_LOBYTE(dxl.DXL_HIWORD(goal_velocity)),
                dxl.DXL_HIBYTE(dxl.DXL_HIWORD(goal_velocity))
            ]
            # Add to syncwrite storage
            addparam_result = self.groupSyncWrite.addParam(motor_id, self.param_goal_velocity)
            if not addparam_result:
                print(f"Failed to add motor ID {motor_id} to SyncWrite")
                quit()

        # Transmit packet to all motors in group (at the same time)
        syncwrite_result = self.groupSyncWrite.txPacket()
        if syncwrite_result != dxl.COMM_SUCCESS:
            print(f"Failed to transmit SyncWrite packet: {self.packetHandler.getTxRxResult(syncwrite_result)}")

        # Clear the syncwrite parameter storage after the transmission
        self.groupSyncWrite.clearParam()
        print("Goal velocities have been broadcasted to motors.")



    def get_position_sync(self):
        present_motor_angles = []
        sync_read_result = self.groupSyncRead.txRxPacket()
        current_t = time.perf_counter()
        if sync_read_result != dxl.COMM_SUCCESS:
            print(f"Failed to sync read packet: {self.packetHandler.getTxRxResult(sync_read_result)}")
        # Retrieve and print the positions for each motor
        a = time.perf_counter()
        index = 0
        for motor_id in self.args.motor_ids:
            if not self.groupSyncRead.isAvailable(motor_id, 132, self.args.LEN_PRO_PRESENT_POSITION):
                print(self.groupSyncRead.isAvailable(motor_id, 132, self.args.LEN_PRO_PRESENT_POSITION),"=============")
                print(f"Data for motor ID {motor_id} is not available")
                continue

            # Read the position (32-bit value)
            present_position = self.groupSyncRead.getData(motor_id, self.args.ADDR_PRESENT_POSITION, self.args.LEN_PRO_PRESENT_POSITION) - self.args.motor_offsets[index]
            
            ## Print out the motor IDs
            # print('motor id', motor_id, 'and position', present_position)
            present_motor_angle = (present_position) * self.motorStep_to_radians
            present_motor_angles.append(present_motor_angle)
            index +=1

            # print(f"Motor {motor_id} present position: {present_position}")
        #print("===========", time.perf_counter()-a)

        return np.array(present_motor_angles), current_t


    def set_current_sync(self, goal_currents):
        for motor_id, current in zip(self.args.motor_ids, goal_currents):
            # Clamp current to safe values
            max_current = 1193  # ≈ 3.21 A for XM430-W210
            current = int(np.clip(current, -max_current, max_current))

            # Convert to 2-byte signed integer (little endian)
            param_goal_current = [
                dxl.DXL_LOBYTE(current & 0xFFFF),
                dxl.DXL_HIBYTE(current & 0xFFFF)
            ]

            # Add param to syncwrite
            success = self.groupSyncWrite.addParam(motor_id, param_goal_current)
            if not success:
                print(f"Failed to add current for motor {motor_id}")
                quit()

        # Transmit to all motors
        result = self.groupSyncWrite.txPacket()
        if result != dxl.COMM_SUCCESS:
            print(f"SyncWrite failed: {self.packetHandler.getTxRxResult(result)}")

        self.groupSyncWrite.clearParam()




    def close_port(self):
        # Close port when done
        self.portHandler.closePort()
