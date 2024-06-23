# 协议格式
协议包含两字节帧头、一字节数据长度 N、N 字节数据，其中数据部分第一字节为功能码。数据以 16 进制高字节在前(设置运动学模型除外)

| 帧头 1 | 帧头 2 | 数据长度 | 功能码 | 数据... | 数据 N |
| :----: | :----: | :------: | :----: | :-----: | :----: |
|  0xFE  |  0xEF  |    N     | 功能码 |   ...   |  ...   |

# 协议数据
## 设置 PID 间隔
请求数据:
| 功能码 |     数据 1-2     |
| :----: | :--------------: |
|  0x00  | PID 间隔时间(ms) |

响应数据: 无

## 设置电机参数
请求数据:
| 功能码 |       数据 1-4       |    数据 5    |
| :----: | :------------------: | :----------: |
|  0x01  | 电机编码器单圈计数值 | 电机是否反转 |

响应数据: 无

## 设置运动学模型
参数均为单精度浮点数，长度四字节。与其他功能不同，该功能为了方便实现，协议中的数据直接从对象中拷贝，实际字节序与硬件平台有关。目前使用的上层板和主控板芯片均为小端序。

可用模型如下:
| 编号  |   模型   |               参数(单位m)                |
| :---: | :------: | :--------------------------------------: |
|   0   | 两轮差速 |            车轮半径、两轮间距            |
|   1   | 三轮全向 |         车轮半径、车轮到中心距离         |
|   2   | 四轮差速 |           车轮半径、左右轮间距           |
|   3   |   麦轮   | 车轮半径、前后轮间距一半、左右轮间距一半 |

请求数据:
| 功能码 |  数据 1  | 数据... |
| :----: | :------: | :-----: |
|  0x02  | 模型编号 |  参数   |

响应数据: 无

## 设置校正系数
请求数据:
| 功能码 |      数据 1-4      |      数据 5-8      |     数据 9-12      |
| :----: | :----------------: | :----------------: | :----------------: |
|  0x03  | X 轴线速度校正系数 | Y 轴线速度校正系数 | Z 轴角速度校正系数 |

响应数据: 无

## 设置速度
请求数据:
| 功能码 |  数据 1-4  |  数据 5-8  | 数据 9-12  |
| :----: | :--------: | :--------: | :--------: |
|  0x04  | X 轴线速度 | Y 轴线速度 | Z 轴角速度 |

响应数据: 无

## 重置里程计
请求数据:
| 功能码 |
| :----: |
|  0x05  |

响应数据: 无

## 获取里程计
请求数据:
| 功能码 |
| :----: |
|  0x06  |

响应数据:
| 功能码 |  数据 1-4  |  数据 5-8  | 数据 9-12  | 数据 13-16 | 数据 17-20 | 数据 21-24 |
| :----: | :--------: | :--------: | :--------: | :--------: | :--------: | :--------: |
|  0x06  | X 轴线速度 | Y 轴线速度 | Z 轴角速度 |  X 轴位置  |  Y 轴位置  | 方向(rad)  |

## 获取 IMU 温度
请求数据:
| 功能码 |
| :----: |
|  0x07  |

响应数据:
| 功能码 | 数据 1-4 |
| :----: | :------: |
|  0x07  |   温度   |

## 获取 IMU 数据
请求数据:
| 功能码 |
| :----: |
|  0x08  |

响应数据，其中加速度单位为重力加速度，角速度为角度制:
| 功能码 |   数据 1-4   |   数据 5-8   |  数据 9-12   | 数据 13-16 | 数据 17-20 | 数据 21-24 |
| :----: | :----------: | :----------: | :----------: | :--------: | :--------: | :--------: |
|  0x08  | X 轴线加速度 | Y 轴线加速度 | Z 轴线加速度 | X 轴角速度 | Y 轴角速度 | Z 轴角速度 |

## 获取超声波雷达距离
请求数据:
| 功能码 |
| :----: |
|  0x09  |

响应数据:
| 功能码 | 数据 1-2 |
| :----: | :------: |
|  0x09  | 距离(mm) |

## 获取电池电压
请求数据:
| 功能码 |
| :----: |
|  0x0A  |

响应数据:
| 功能码 | 数据 1-4 |
| :----: | :------: |
|  0x0A  |   电压   |