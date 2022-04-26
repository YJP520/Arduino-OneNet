/*
 *********************************************************
 * TIME       :     2022.04.06  ~ 2022.04.25
 * Author     :     YU.J.P
 * Project    :     Vehicle ARDUINO UNO + ESP8266 + ONENET
 *********************************************************
 * V2.0.0   --  2022.04.11  --  部署 DHT11 & MICS-6814 传感器
 * V2.0.1   --  2022.04.12  --  部署 ESP8266 WIFI透传
 * V2.1.1   --  2022.04.13  --  部署 GPS 模块
 * V2.1.2   --  2022.04.14  --  优化软件结构
 * V2.1.3   --  2022.04.16  --  修复设备冲突BUG & 修复GPS读取BUG & 优化软件结构
 * V2.1.4   --  2022.04.17  --  更新json格式数据 & 优化代码
 * V2.2.5   --  2022.04.24  --  更新json格式数据打包传送 & 优化代码
 * V2.2.6   --  2022.04.25  --  硬件完善 & 可视化界面优化调试
 *  
 *********************************************************
 * ESP8266    :   
 *        RXD   -->   TXD   Digital Pin 1
 *        TXD   -->   RXD   Digital Pin 0
 *        3.3V  -->   3.3V 
 *        GND   -->   GND
 *********************************************************
 * DHT11      :     
 *        DATA  -->   Digital Pin 2  
 *********************************************************
 * MICS-6814  :
 *        +5V   -->   +5V
 *        GND   -->   GND
 *        NO2   -->   Digital Pin 4
 *        NH3   -->   Digital Pin 5
 *        CO    -->   Digital Pin 3
 *********************************************************
 * GPS        :
 *        +5V   -->   +5V 
 *        GND   -->   GND
 *        RXD   -->   TX Digital Pin 7
 *        TXD   -->   RX Digital Pin 6
 *********************************************************
 * RGB        :
 *        GND   -->   GND
 *        R     -->   Digital Pin 13
 *        G     -->   Digital Pin 12
 *        B     -->   Digital Pin 11
 *********************************************************
 * BEEP       :
 *        GND   -->   GND
 *        S     -->   Digital Pin 10
 *
 *********************************************************
 */

/* INCLUDE */
#include <TinyGPS++.h>            //GPS解析
#include<SoftwareSerial.h>        //软串口实现通信
#include <ArduinoJson.h>          //使用JSON库合成JSON数据

#include "edp.c"                  //ONENET协议
#include "DHT11.h"                //温湿度传感器头文件
#include "MICS6814.h"             //MICS-6814多种气体传感器头文件

/* 引脚定义 */
#define   PIN_CO            3           //接 Digital Pin 3
#define   PIN_NO2           4           //接 Digital Pin 4
#define   PIN_NH3           5           //接 Digital Pin 5
#define   DHT11PIN          2           //接 Digital Pin 2
static const int RXPin = 6, TXPin = 7;  //接 Digital Pin 6 7 GPS

/*ONENET SETTING */
#define KEY         "7qkkcwip12Jk6ayo327VNHLut2I="    //APIkey 
#define ID          "928286502"                       //设备ID
#define PUSH_ID     "505862"                          //产品ID

/* New Defination */
#define _baudrate       115200                        //串口波特率
#define WIFI_UART       Serial
static const uint32_t   GPSBaud = 9600;               //GPS软串口波特率
SoftwareSerial softSerial1(RXPin, TXPin);             // The serial connection to the GPS device

TinyGPSPlus     gps;                                  // The TinyGPS++ object
//StaticJsonDocument<200> doc;                          //声明一个JsonDocument对象
DHT11           dht11;                                //创建DHT11对象
MICS6814        gas(PIN_CO, PIN_NO2, PIN_NH3);        //创建MICS6814对象

int circle = 50;                                      //循环变量

/*********************** ESP8266 BEGIN ******************************************************************/

/* 初始化ESP8266 */
void Init_esp8266()
{
  while (!doCmdOk("AT", "OK"));                                           //检查工作状态
  while (!doCmdOk(F("AT+CWMODE=3"), "OK"));                               //工作模式
  while (!doCmdOk(F("AT+CWJAP=\"YJP\",\"14159265358979\""), "OK"));       //设置WIFI连接
  while (!doCmdOk(F("AT+CIPSTART=\"TCP\",\"183.230.40.39\",876"), "OK")); //这个是EDP的服务器和端口
  while (!doCmdOk(F("AT+CIPMODE=1"), "OK"));                              //透传模式
  while (!doCmdOk(F("AT+CIPSEND"), ">"));                                 //开始发送
}

/*
* doCmdOk
* 发送命令至模块，从回复中获取期待的关键字
* keyword: 所期待的关键字
* 成功找到关键字返回true，否则返回false
*/
bool doCmdOk(String data, char *keyword)
{
  bool result = false;
  if (data != "")                     //对于tcp连接命令，直接等待第二次回复
    WIFI_UART.println(data);          //发送AT指令
  if (data == "AT")                   //检查模块存在
    delay(1000);
  else
    while (!WIFI_UART.available());   //等待模块回复
      delay(200);
  if (WIFI_UART.find(keyword))        //返回值判断
    result = true;
  else  
    result = false;
  while (WIFI_UART.available()) 
    WIFI_UART.read();                 //清空串口接收缓存
  delay(500);                         //指令时间间隔
  return result;
}

/*********************** ESP8266 END ********************************************************************/

/*********************** SETUP BEGIN ********************************************************************/
void setup()
{
  pinMode(10,OUTPUT);                 //蜂鸣器PWM
  
  tone(10,784);   //频率, 单位Hz       //蜂鸣器鸣响 开机成功
  delay(500);    //等待500毫秒
  noTone(10);     //停止发声             
  
  WIFI_UART.begin( _baudrate );       //设置波特率
  softSerial1.begin(9600);            //初始化软串口通信；
  softSerial1.listen();               //监听软串口通信

  GPS_setup();                        //GPS信息打印
  //gas.calibrate();                    //校正MICS6814程序
  digitalWrite(12, HIGH);             //蓝灯亮 表示传感器正常

  Init_esp8266();                     //esp8266初始化
  digitalWrite(11, HIGH);             //绿灯亮 表示esp8266连接正常

  /* 蜂鸣器鸣响 网络连接成功 */
  tone(10,784);delay(200);noTone(10);delay(100);
  tone(10,784);delay(200);noTone(10); 
}

/*********************** SETUP END **********************************************************************/

/*********************** LOOP BEGIN *********************************************************************/
void loop()
{
  /* 变量声明 */
  static int edp_connect = 0;                       //
  edp_pkt rcv_pkt;                                  //
  unsigned char pkt_type;                           //
  int tmp;                                          //
  
  int   Temperature,Humidity;                       //温度 & 湿度
  float CO_value,NO2_value,NH3_value,SO2_value;     //CO NO2 NH3 SO2
  float Longitude,Latitude;                         //经度 & 纬度
  char Lon[20],Lat[20];                             //转换字符串 解决精度不够问题
  StaticJsonDocument<200> doc;                      //声明一个JsonDocument对象
  
  /* EDP 连接 */
  if (!edp_connect)
  {
    while (WIFI_UART.available()) WIFI_UART.read(); //清空串口接收缓存
    packetSend(packetConnect(ID, KEY));             //发送EPD连接包
    while (!WIFI_UART.available());                 //等待EDP连接应答
    if ((tmp = WIFI_UART.readBytes(rcv_pkt.data, sizeof(rcv_pkt.data))) > 0 )
      if (rcv_pkt.data[0] == 0x20 && rcv_pkt.data[2] == 0x00 && rcv_pkt.data[3] == 0x00)
      {
        edp_connect = 1;
        digitalWrite(13, LOW);                      //使Led灭
      }
    packetClear(&rcv_pkt);
  }

  /* 数据获取与发送 */
  circle++;                                           //50周期发送一次数据
  if(circle > 50 && edp_connect)
  {
    circle = 0;                                       //变量清零

    //smartDelay(1000);                                 //读取GPS
    
    //dht11.read(DHT11PIN);
    //Temperature = dht11.temperature;                  //获得温度
    //Humidity = dht11.humidity;                        //获得湿度
    randomSeed(analogRead(A5));  //使用一个未被使用到的引脚A5
    Temperature = 15 + random(25);
    Humidity = 40 + random(25);
    
    //CO_value = gas.measure(CO);                       //获得CO浓度
    //NO2_value = gas.measure(NO2);                     //获得NO2浓度
    //NH3_value = gas.measure(NH3);                     //获得NH3浓度
    CO_value = 6.19 + random(100) / 100.0;
    NO2_value = 0.10 + random(100) / 100.0;
    NH3_value = 1.12 + random(100) / 100.0;
    SO2_value = 0 + random(10) / 100.0;

    //Longitude = gps.location.lng();                   //获得经度
    //Latitude = gps.location.lat();                    //获得纬度
    Longitude = 106.524349 + random(100) / 10000.0;
    Latitude = 29.458008 + random(100) / 10000.0;
    dtostrf(Longitude,3,6,Lon);                       //float型转换成char型
    dtostrf(Latitude,2,6,Lat);                        //float型转换成char型

    JsonObject Reported = doc.createNestedObject("reported"); //温湿度
    JsonObject Desired = doc.createNestedObject("desired");   //气体浓度
    JsonObject Location = doc.createNestedObject("Location"); //位置信息
    
    Reported["Temperature"] = Temperature;
    Reported["Humidity"] = Humidity;
    Desired["CO_value"] = CO_value;
    Desired["NO2_value"] = NO2_value;
    Desired["NH3_value"] = NH3_value;
    Desired["SO2_value"] = SO2_value;
    Location["lon"] = Lon;
    Location["lat"] = Lat;

    String data = doc.as<String>();
    const char* json_out = data.c_str();
    
    //char* json_out = "{\"value\":{\"lon\":106.524349,\"lat\":29.458008}}";
    digitalWrite(13, HIGH);                                 //红灯亮 发送信息
    packetSend(PacketSavedataJson(json_out));               //将新数据值上传至数据流
    delay(200);
    digitalWrite(13, LOW);                                  //红灯灭 发送信息完成
  }

  /* 串口接收信息 作出反馈 */
  while (WIFI_UART.available())
  {
    readEdpPkt(&rcv_pkt);
    if (isEdpPkt(&rcv_pkt))
    {
      pkt_type = rcv_pkt.data[0];
      switch (pkt_type)
      {
        case CMDREQ:
          char edp_command[50];
          char edp_cmd_id[40];
          long id_len, cmd_len, rm_len;
          char datastr[20];
          char val[10];
          memset(edp_command, 0, sizeof(edp_command));
          memset(edp_cmd_id, 0, sizeof(edp_cmd_id));
          edpCommandReqParse(&rcv_pkt, edp_cmd_id, edp_command, &rm_len, &id_len, &cmd_len);  
          
          //数据处理与应用中EDP命令内容对应
          //本例中格式为  datastream:[1/0] 
          sscanf(edp_command, "%[^:]:%s", datastr, val);
          
          if (atoi(val) == 1)
            digitalWrite(13, HIGH);     // 使Led亮
          else
            digitalWrite(13, LOW);      // 使Led灭
          
          if(atoi(val) > 1)
          {
             if(atoi(val) >= 100 && atoi(val) <=1000)
             {   
                //
             }
             else  if(atoi(val) >= 10 && atoi(val) <=100)
             {  
                //
             }
             else  if(atoi(val) > 0 && atoi(val) <=10)
             {
                //
             }
             delay(200);  
          }                              
          //pin high.  Stop receiving data
          //packetSend(packetDataSaveTrans(NULL, datastr, val)); //将新数据值上传至数据流
          break;
        default:;break;
      }
    }
  }

  /* 清包 */
  if(rcv_pkt.len > 0)
    packetClear(&rcv_pkt);
  delay(150);
}

/*********************** LOOP END ***********************************************************************/

/*********************** RX TX BEGIN ********************************************************************/
/*
* readEdpPkt
* 从串口缓存中读数据到接收缓存
*/
bool readEdpPkt(edp_pkt *p)
{
  int tmp;
  if ((tmp = WIFI_UART.readBytes(p->data + p->len, sizeof(p->data))) > 0 )
  {
//    rcvDebug(p->data + p->len, tmp);
    p->len += tmp;
  }
  return true;
}
 
/*
* packetSend
* 将待发数据发送至串口，并释放到动态分配的内存
*/
void packetSend(edp_pkt* pkt)
{
  if (pkt != NULL)
  {
    WIFI_UART.write(pkt->data, pkt->len);     //串口发送
    WIFI_UART.flush();
    free(pkt);                                //回收内存
  }
}

/*********************** RX TX END **********************************************************************/

/*********************** GPS BEGIN **********************************************************************/

/* GPS SETUP */
void GPS_setup()
{
  Serial.print(F("Testing TinyGPS++ library v. "));
  Serial.println(TinyGPSPlus::libraryVersion()); 
}

/*
* This custom version of delay() ensures that the gps object
* is being "fed". 读取信息前接收处理
*/
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (softSerial1.available())
      gps.encode(softSerial1.read());
  } while (millis() - start < ms);
}

/*********************** GPS END ************************************************************************/
