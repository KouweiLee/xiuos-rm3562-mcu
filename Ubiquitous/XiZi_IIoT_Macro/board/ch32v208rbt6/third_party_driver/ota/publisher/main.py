import paho.mqtt.client as mqtt
import json
import os
import hashlib

# 配置参数（根据实际情况修改）
MQTT_BROKER = "47.115.50.232"  # MQTT代理地址
MQTT_PORT = 1883                   # 端口
FILE_PATH = "XiZi-ch32v208rbt6-app.bin"         # 要发送的固件文件路径
CLIENT_ID = "D001"                 # 设备客户端ID
VERSION = "001.000.002"            # 新固件版本号
FILE_ID = 1                        # 文件标识符

def calculate_md5(file_path):
    """ 计算文件的MD5哈希值 """
    hash_md5 = hashlib.md5()
    with open(file_path, "rb") as f:
        for chunk in iter(lambda: f.read(4096), b""):
            hash_md5.update(chunk)
    return hash_md5.hexdigest()

def compact_json(payload: dict) -> str:
    """生成无空格/换行的紧凑JSON字符串"""
    return json.dumps(payload, separators=(',', ':'))

def on_connect(client, userdata, flags, rc):
    """ 连接回调函数 """
    if rc == 0:
        print("连接成功!")
        # 订阅文件请求主题
        client.subscribe("xiuosiot/ota/files")
        
        # 发送初始更新信息
        init_payload = {
            "fileSize": userdata["file_size"],
            "version": userdata["fw_version"],
            "fileId": userdata["file_id"],
            "md5": userdata["file_md5"]
        }
        client.publish(
            topic=f"ota/{CLIENT_ID}/update",
            payload=compact_json(init_payload),
            qos=1
        )
        print(f"已发送初始化消息到 ota/{CLIENT_ID}/update")
        
    else:
        print(f"连接失败，错误码：{rc}")

def on_message(client, userdata, msg):
    """ 消息到达回调函数 """
    try:
        payload = json.loads(msg.payload.decode())
        print("收到文件请求:", payload)

        # 验证必要字段
        required_fields = ["clientId", "fileId", "fileOffset", "size"]
        if not all(field in payload for field in required_fields):
            print("错误：缺少必要字段")
            return

        # 验证客户端ID
        if payload["clientId"] != CLIENT_ID:
            print(f"忽略非目标客户端请求：{payload['clientId']}")
            return

        # 提取请求参数
        offset = payload["fileOffset"]
        request_size = payload["size"]
        reply_topic = f"ota/{CLIENT_ID}/files"

        # 读取文件内容
        try:
            with open(FILE_PATH, "rb") as f:
                f.seek(offset)
                file_data = f.read(request_size)
                
                if len(file_data) == 0:
                    print("错误：读取到空数据或超出文件范围")
                    return
        except Exception as e:
            print(f"文件读取失败：{str(e)}")
            return

        # 发送文件片段
        client.publish(reply_topic, file_data, qos=1)
        print(f"已发送 {len(file_data)} 字节到 {reply_topic}")

    except json.JSONDecodeError:
        print("错误：无效的JSON格式")
    except Exception as e:
        print(f"处理消息时出错：{str(e)}")

def main():
    # 验证文件存在性
    if not os.path.exists(FILE_PATH):
        print(f"错误：文件 {FILE_PATH} 不存在")
        exit(1)

    # 计算文件参数
    file_size = os.path.getsize(FILE_PATH)
    file_md5 = calculate_md5(FILE_PATH)

    # 创建MQTT客户端
    client = mqtt.Client(userdata={
        "file_size": file_size,
        "fw_version": VERSION,
        "file_id": FILE_ID,
        "file_md5": file_md5
    })

    # 设置回调函数
    client.on_connect = on_connect
    client.on_message = on_message

    try:
        client.username_pw_set("ch32v208", "xiuosiot")  # 取消注释并填写凭证
        client.connect(MQTT_BROKER, MQTT_PORT, 60)
        client.loop_forever()
    except KeyboardInterrupt:
        print("\n程序已终止")
    except Exception as e:
        print(f"连接错误：{str(e)}")
        exit(1)

if __name__ == "__main__":
    main()
