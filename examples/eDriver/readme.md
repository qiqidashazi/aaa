
/**********************************************************************************/

# eDriver主站使用说明

# 主站配置的从机的RPDO: 202 303 404 ，发送 TPD:  182 283 384
#                         ^   ^  ^                 `    `   ` 
# 主站与其相反：TPDO是  202 303 404，       RPDO: 182 283 384

# 只需要保证每个从机RPDO里有主站发送的对应TPDO的COBID就行

# PDO模式：
1.  MInitialize_eDriver ：进行主站的初始化 -> 配置伺服驱动 -> 启动同步
2.  MBootServoDriver ：伺服准备好 -> 等待打开伺服使能 -> 伺服运行 -> 激活插补模式
3.  MPDOSetPosition:   控制位置


# SDO模式
1.  MInitialize_eDriver ：进行主站的初始化 -> 配置伺服驱动
2.  MSetEDriverControl 启动节点-激活插补模式
3.  MSetPosition 设置位置信息

MGetServoStatus() 调试用

# objDictionary -slave.c/.h 该文件是在我开发主站时，对从站进行配置模拟出的文件。编译时请保证该文件不在编译目录下。
# 该源文件中部分变量是不使用的，代码部分几乎近似主站字典。只是在使用SDO进行网络通信后会改成client端的文件，既对应的参数


/**********************************************************************************/
