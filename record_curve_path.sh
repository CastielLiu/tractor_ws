#曲线记录
rosservice call /record_path_service "command_type: 1  
path_file_name: '1_101.txt'
path_type: 101"

#曲线停止
rosservice call /record_path_service "command_type: 2  
path_file_name: '1_101.txt'
path_type: 101"

#顶点型记录
rosservice call /record_path_service "command_type: 1  
path_file_name: '1_100.txt'
path_type: 100"

#记录当前点
rosservice call /record_path_service "command_type: 3  
path_file_name: '1_100.txt'
path_type: 100"

#停止
rosservice call /record_path_service "command_type: 3  
path_file_name: '1_100.txt'
path_type: 100"



#自动驾驶开始
rosservice call /driverless_service "command_type: 1
path_file_name: '1_101.txt'
path_type: 101
speed: 0.0"


#自动驾驶暂停
rosservice call /driverless_service "command_type: 3
path_file_name: '1_101.txt'
path_type: 101
speed: 0.0"

#自动驾驶停止
rosservice call /driverless_service "command_type: 2
path_file_name: '1_101.txt'
path_type: 101
speed: 0.0"

#确认停止
rosservice call /driverless_service "command_type: 4
path_file_name: '1_101.txt'
path_type: 101
speed: 0.0"







