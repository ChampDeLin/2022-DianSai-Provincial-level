from Maix import MIC_ARRAY as mic
from Maix import GPIO
from Maix import FPIOA
import lcd,time
import math
import utime
from machine import UART,Timer
from fpioa_manager import fm

#*****************************************GPIO********************************#
fpioa = FPIOA()
fpioa.set_function(30, fpioa.GPIOHS0)
fpioa.set_function(31, fpioa.GPIOHS1)
fpioa.set_function(32, fpioa.GPIOHS2)
fpioa.set_function(33, fpioa.GPIOHS3)
fpioa.set_function(34, fpioa.GPIOHS4)
fpioa.set_function(16, fpioa.GPIOHS5)
fpioa.set_function(28, fpioa.GPIOHS6)
key_1 = GPIO(GPIO.GPIOHS0, GPIO.IN, GPIO.PULL_UP)
key_2 = GPIO(GPIO.GPIOHS1, GPIO.IN, GPIO.PULL_UP)
key_3 = GPIO(GPIO.GPIOHS2, GPIO.IN, GPIO.PULL_UP)
key_4 = GPIO(GPIO.GPIOHS3, GPIO.IN, GPIO.PULL_UP)
key_5 = GPIO(GPIO.GPIOHS4, GPIO.IN, GPIO.PULL_UP)
key_6 = GPIO(GPIO.GPIOHS5, GPIO.IN, GPIO.PULL_UP)
key_7 = GPIO(GPIO.GPIOHS6, GPIO.IN, GPIO.PULL_UP)
print("GPIO_init!")


#*****************************************DUOJI********************************#
#该函数将角度转换为步进电机的位置参数
#假定角度范围是0-180
def yiding_yimu(angle):
    return int(4096 * (angle / 360.0))

#该函数生成控制帧
def create_control_message(angle):
    step = yiding_yimu(angle)
    lw = step % 256
    hg = step>>8
    buffer = bytearray(7)
    buffer[0]=0x2A
    buffer[1]=lw
    buffer[2]=hg
    buffer[3]=0
    buffer[4]=0
    buffer[5]=0xE8 #这里没有做调速的逻辑
    buffer[6]=0x03 #直接将选定的参数写在代码里，可按需修改
    return SendBuffer(7, buffer, 0x03)

#该函数根据角度调用相应函数生成控制帧
#然后通过uart对象发送控制帧
def control(uart, angle):
    uart.write(create_control_message(angle))
    uart.read()

def SendBuffer(length, buffer, cmd):
    data = bytearray(length + 6)
    data[0] = 0xff
    data[1] = 0xff
    data[2] = 0x01
    data[3] = length + 2
    data[4] = cmd
    i = 0
    sum = 0
    while  i < length:
        data[5 + i] = buffer[i]
        sum += buffer[i]
        i += 1
    data[length + 5] = ~(data[2] + data[3] + data[4] + sum)
    return data

#引脚映射
fm.register(6, fm.fpioa.UART1_RX, force=True)
fm.register(7, fm.fpioa.UART1_TX, force=True)

def laser_Init():
    fm.register(35, fm.fpioa.GPIO0)
    laser = GPIO(GPIO.GPIO0, GPIO.OUT)
    return laser

def set_laser(laser, key):
    if(key == 0):
        laser.value(0)
    else:
        laser.value(1)

laser = laser_Init()

#while True:

    #set_laser(laser, 1)#点亮激光
    #utime.sleep(3)
    #set_laser(laser, 0)#关闭激光
    #utime.sleep(3)

#构造函数
uart = UART(UART.UART1, 115200, read_buf_len=4096)

##舵机先归零，然后遍历半个圆周，最后归零
##测试用例，舵机应当按照描述工作
#set_laser(laser, 1)#点亮激光
#utime.sleep(2)
#set_laser(laser, 0)#关闭激光
#utime.sleep(1)
#set_laser(laser, 1)#点亮激光


#while True:
    #control(uart, 90)
    #utime.sleep(2)


    #for i in range(3):
        #th = i * 10.0
        #th = 90 - th
        #control(uart, th)
        #utime.sleep(2)

    #control(uart, 90)

    #for i in range(3):
        #th = i * -10.0
        #th = 90 - th
        #control(uart, th)
        #utime.sleep(2)

    #control(uart, 90)



#*****************************************Maike********************************#
#import maix_vision
lcd.init()
mic.init()
#mic.init(i2s_d0=34, i2s_d1=8, i2s_d2=33, i2s_d3=9, i2s_ws=32, i2s_sclk=10,\
            #sk9822_dat=7, sk9822_clk=35)#可自定义配置 IO
#mic.init(i2s_d0=23, i2s_d1=22, i2s_d2=21, i2s_d3=20, i2s_ws=19, i2s_sclk=18, sk9822_dat=24, sk9822_clk=25)
#mic.init(i2s_d0=20, i2s_d1=21, i2s_d2=15, i2s_d3=8, i2s_ws=7, i2s_sclk=6, sk9822_dat=25, sk9822_clk=24)# for maix cube

meancount = 100
mean_x =[]
meanwriteindex = 0
meanfound = False
meanlost = 0

x230 = [0]

def set_mean_buffer(buffercount):
    global meancount

    meancount = buffercount

    global mean_x
    mean_x.clear()
    for i in range(meancount):
        mean_x.append(0);

#缺省200过滤
#set_mean_buffer(200)

x2cm_k_upper_150to75 = 2.272727
#x2cm_a_upper_150to75 = 3.027644
x2cm_k_upper_75to0 = 2.577319
#x2cm_a_upper_75to0 = 3.027644
x2cm_k_lower_75to0 = 3.378318
#x2cm_a_lower_75to0 = 3.378318
x2cm_k_lower_150to75 = 2.161383
#x2cm_a_lower_150to75 = 3.034835
x2cm_x150 = 57
x2cm_x75 = 27.29
x2cm_x0 = 3
x2cm_Fx75 = -20.9
x2cm_Fx150 = -57

#接收设备中点到音源中心的距离,cm
chuizhichangdu = 275

def ConvertToDensity(getdata):
    if getdata > x2cm_x150:
        return 150
    elif getdata > x2cm_x75:
        return (x2cm_k_upper_150to75 * (getdata - x2cm_x75)) + 75
    elif getdata > x2cm_x0:
        return x2cm_k_upper_75to0 *getdata
    elif getdata > x2cm_Fx75:
        return x2cm_k_lower_75to0 *getdata
    elif getdata > x2cm_Fx150:
        return (x2cm_k_lower_150to75 * (getdata - x2cm_Fx75)) - 75
    #elif getdata <= x2cm_Fx150:
    else:
        return -150



#根据阈值查找
def get_mic_dir(xresult):
    global meanfound
    global mean_x
    global meanwriteindex
    global meancount
    global meanlost
    global chuizhichangdu
    global x230
    global x2cm_k_upper
    global x2cm_a_upper
    global x2cm_k_lower
    global x2cm_a_lower
    global x2cm_x0


    okcount = 0

    imga = mic.get_map()    # 获取声音源分布图像
    imgb = imga.resize(160, 160)
    #imgc = imgb.to_rainbow(1)
    #lcd.display(imgc)
    #b = mic.get_dir(imga)   # 计算、获取声源方向
    #print('====================');
    #for y in range(imgb.height()):
        #for x in range(imgb.width()):
            #print('%03d'%imgb.get_pixel(x, y), end=" ")
        #print('\r\n', end=" ")
    #return

    tempx = 0

    #th = [(230, 255)]
    th = [(230, 255)]
    #blobs = imgb.find_blobs(th, area_threshold=16, pixels_threshold=1, merge=True, x_stride=1, y_stride=1)
    blobs = imgb.find_blobs(th, area_threshold=16, pixels_threshold=1, merge=True, x_stride=1, y_stride=1)
    #print(blobs)
    if blobs:
        tempx = blobs[0].cx() - 80
        okcount = okcount + 1
    else:
        tempx = 0

    th = [(10, 255)]
    #blobs = imgb.find_blobs(th, area_threshold=16, pixels_threshold=1, merge=True, x_stride=1, y_stride=1)
    blobs = imgb.find_blobs(th, area_threshold=16, pixels_threshold=1, merge=True, x_stride=1, y_stride=1)

    if blobs:
        tempx = tempx + blobs[0].cx() - 80
        okcount = okcount + 1

    if okcount > 0:
        xresult[0] = tempx / okcount
        return True
    else:
        return False


def test2():
    global meanfound
    global mean_x
    global meanwriteindex
    global meancount
    global meanlost
    global chuizhichangdu
    global x230
    global x2cm_k_upper
    global x2cm_a_upper
    global x2cm_k_lower
    global x2cm_a_lower
    global x2cm_x0

    lcd.clear()
    lcd.draw_string(10, 10, "Test2:", lcd.YELLOW, lcd.BLACK)

    #设置过滤缓冲300
    set_mean_buffer(50)

    meanwriteindex = 0
    meanfound = False

    while True:
        if get_mic_dir(x230) == True:
            meanlost = 0
            mean_x[meanwriteindex] = x230[0]
            meanwriteindex = meanwriteindex + 1
            if meanfound == False:#第一次满
                if meanwriteindex >= meancount:
                    meanfound = True
            meanwriteindex = meanwriteindex % meancount


            #如果满了就可以计算了
            if meanfound == True:
                allv = 0.0
                #print(mean_x)
                max_num = mean_x[0]
                min_num = mean_x[0]
                for i in range(meancount):
                    if mean_x[i] >  max_num:
                        max_num = mean_x[i]
                    if mean_x[i] < min_num:
                        min_num = mean_x[i]

                    allv = allv + mean_x[i]
                allv = allv - max_num - min_num
                allv = (allv / (meancount-2))

                #x = allv * 2.4916943521594684385382059800664
                #if allv >= x2cm_x0:
                    #x = x2cm_k_upper*allv + x2cm_a_upper
                #else:
                    #x = x2cm_k_lower*allv + x2cm_a_lower
                x = ConvertToDensity(allv)

                r = math.sqrt(x*x + chuizhichangdu*chuizhichangdu)
                th = math.atan(x/chuizhichangdu)*180 / math.pi

                print("{" + str(allv) + ";" + str(x)+";" + str(r) + ";" + str(th) + "}")
                lcd.draw_string(10, 30, "rawx:%0.1f; x:%0.1f        "% (allv ,x), lcd.YELLOW, lcd.BLACK)
                lcd.draw_string(10, 60, "L:%0.1f; Theta:%0.1f        "%(r, th), lcd.YELLOW, lcd.BLACK)

            else:
                print("WAIT FULL")
        else:#如果是无效帧
            meanlost = meanlost + 1
            if meanlost > 10:
                meanfound = False

        #如果按了退出键
        if key_4.value() == 0:
            utime.sleep_ms(30)
            if key_4.value() == 0:
                while key_4.value() == 0:
                    pass
                break


def test3():
    global meanfound
    global mean_x
    global meanwriteindex
    global meancount
    global meanlost
    global chuizhichangdu
    global x230
    global x2cm_k_upper
    global x2cm_a_upper
    global x2cm_k_lower
    global x2cm_a_lower
    global x2cm_x0

    lcd.clear()
    lcd.draw_string(10, 10, "Test3:", lcd.YELLOW, lcd.BLACK)

    #设置过滤缓冲300
    set_mean_buffer(100)

    meanwriteindex = 0
    meanfound = False

    set_laser(laser, 0)#点亮激光

    while True:
        if get_mic_dir(x230) == True:
            meanlost = 0
            mean_x[meanwriteindex] = x230[0]
            meanwriteindex = meanwriteindex + 1
            if meanfound == False:#第一次满
                if meanwriteindex >= meancount:
                    meanfound = True
            meanwriteindex = meanwriteindex % meancount


            #如果满了就可以计算了
            if meanfound == True:
                allv = 0.0
                #print(mean_x)
                max_num = mean_x[0]
                min_num = mean_x[0]
                for i in range(meancount):
                    if mean_x[i] >  max_num:
                        max_num = mean_x[i]
                    if mean_x[i] < min_num:
                        min_num = mean_x[i]

                    allv = allv + mean_x[i]
                allv = allv - max_num - min_num
                allv = (allv / (meancount-2))

                #x = allv * 2.4916943521594684385382059800664
                #if allv >= x2cm_x0:
                    #x = x2cm_k_upper*allv + x2cm_a_upper
                #else:
                    #x = x2cm_k_lower*allv + x2cm_a_lower
                x = ConvertToDensity(allv)

                r = math.sqrt(x*x + chuizhichangdu*chuizhichangdu)
                th = math.atan(x/chuizhichangdu)*180 / math.pi
                #thcontrol = math.atan(x/255)*180 / math.pi
                if x<=60 and x>=-60:
                    thcontrol = math.atan(x/255)*180 / math.pi
                elif x<-60:
                    thcontrol = math.atan(x/270)*180 / math.pi
                else:
                    thcontrol = math.atan(x/280)*180 / math.pi

                print("{" + str(allv) + ";" + str(x)+";" + str(r) + ";" + str(th) + "}")
                lcd.draw_string(10, 30, "rawx:%0.1f; x:%0.1f        "% (allv ,x), lcd.YELLOW, lcd.BLACK)
                lcd.draw_string(10, 60, "L:%0.1f; Theta:%0.1f        "%(r, th), lcd.YELLOW, lcd.BLACK)

                thcontrol = 90 - thcontrol

                control(uart, thcontrol)
                set_laser(laser, 1)#点亮激光
                #time.sleep_ms(2000) #等待舵机运行到指定位置
                #time.sleep_ms(5000) #在当前位置等待5秒
                #control(uart, 180) #舵机归位
                #time.sleep_ms(2000) #等待舵机归位


            else:
                print("WAIT FULL")
        else:#如果是无效帧
            meanlost = meanlost + 1
            if meanlost > 10:
                meanfound = False

        #如果按了退出键
        if key_4.value() == 0:
            utime.sleep_ms(30)
            if key_4.value() == 0:
                while key_4.value() == 0:
                    pass
                set_laser(laser, 0)#关闭激光
                break

def test4():
    global meanfound
    global mean_x
    global meanwriteindex
    global meancount
    global meanlost
    global chuizhichangdu
    global x230
    global x2cm_k_upper
    global x2cm_a_upper
    global x2cm_k_lower
    global x2cm_a_lower
    global x2cm_x0

    lcd.clear()
    lcd.draw_string(10, 10, "Test4:", lcd.YELLOW, lcd.BLACK)

    #设置过滤缓冲300
    set_mean_buffer(10)

    meanwriteindex = 0
    meanfound = False

    set_laser(laser, 1)#点亮激光

    while True:
        if get_mic_dir(x230) == True:
            meanlost = 0
            mean_x[meanwriteindex] = x230[0]
            meanwriteindex = meanwriteindex + 1
            if meanfound == False:#第一次满
                if meanwriteindex >= meancount:
                    meanfound = True
            meanwriteindex = meanwriteindex % meancount


            #如果满了就可以计算了
            if meanfound == True:
                allv = 0.0
                #print(mean_x)
                max_num = mean_x[0]
                min_num = mean_x[0]
                for i in range(meancount):
                    if mean_x[i] >  max_num:
                        max_num = mean_x[i]
                    if mean_x[i] < min_num:
                        min_num = mean_x[i]

                    allv = allv + mean_x[i]
                allv = allv - max_num - min_num
                allv = (allv / (meancount-2))

                #x = allv * 2.4916943521594684385382059800664
                #if allv >= x2cm_x0:
                    #x = x2cm_k_upper*allv + x2cm_a_upper
                #else:
                    #x = x2cm_k_lower*allv + x2cm_a_lower
                x = ConvertToDensity(allv)

                r = math.sqrt(x*x + chuizhichangdu*chuizhichangdu)
                th = math.atan(x/chuizhichangdu)*180 / math.pi

                if x<=60 and x>=-60:
                    thcontrol = math.atan(x/255)*180 / math.pi
                elif x<-60:
                    thcontrol = math.atan(x/270)*180 / math.pi
                else:
                    thcontrol = math.atan(x/280)*180 / math.pi

                #thcontrol = math.atan(x/255)*180 / math.pi

                print("{" + str(allv) + ";" + str(x)+";" + str(r) + ";" + str(th) + ";"+ str(thcontrol) + "}")
                lcd.draw_string(10, 30, "rawx:%0.1f; x:%0.1f        "% (allv ,x), lcd.YELLOW, lcd.BLACK)
                lcd.draw_string(10, 60, "L:%0.1f; Theta:%0.1f        "%(r, th), lcd.YELLOW, lcd.BLACK)

                thcontrol = 90 - thcontrol

                control(uart, thcontrol)
                #time.sleep_ms(2000) #等待舵机运行到指定位置
                #time.sleep_ms(5000) #在当前位置等待5秒
                #control(uart, 180) #舵机归位
                #time.sleep_ms(2000) #等待舵机归位


            else:
                print("WAIT FULL")
        else:#如果是无效帧
            meanlost = meanlost + 1
            if meanlost > 10:
                meanfound = False

        #如果按了退出键
        if key_4.value() == 0:
            utime.sleep_ms(30)
            if key_4.value() == 0:
                while key_4.value() == 0:
                    pass
                set_laser(laser, 0)#关闭激光
                break
def jiaozhun():
    global meanfound
    global mean_x
    global meanwriteindex
    global meancount
    global meanlost
    global chuizhichangdu
    global x230
    global x2cm_k_upper_150to75
    #global x2cm_a_upper_150to75
    global x2cm_k_upper_75to0
    #global x2cm_a_upper_75to0
    global x2cm_k_lower_75to0
    #global x2cm_a_lower_75to0
    global x2cm_k_lower_150to75
    #global x2cm_a_lower_150to75
    global x2cm_x150
    global x2cm_x75
    global x2cm_x0
    global x2cm_Fx75
    global x2cm_Fx150

    lcd.clear()
    lcd.draw_string(10, 10, "JiaoZhun:", lcd.YELLOW, lcd.BLACK)

    #设置过滤缓冲300
    set_mean_buffer(150)

    meanwriteindex = 0
    meanfound = False

    #xupper150 = 0
    #xupper75 = 0
    #x0 = 0
    #xlower75 = 0
    #xlower150 = 0

    allv = 0

    while True:
        if get_mic_dir(x230) == True:
            meanlost = 0
            mean_x[meanwriteindex] = x230[0]
            meanwriteindex = meanwriteindex + 1
            if meanfound == False:#第一次满
                if meanwriteindex >= meancount:
                    meanfound = True
            meanwriteindex = meanwriteindex % meancount


            #如果满了就可以计算了
            if meanfound == True:
                allv = 0.0
                #print(mean_x)
                max_num = mean_x[0]
                min_num = mean_x[0]
                for i in range(meancount):
                    if mean_x[i] >  max_num:
                        max_num = mean_x[i]
                    if mean_x[i] < min_num:
                        min_num = mean_x[i]

                    allv = allv + mean_x[i]
                allv = allv - max_num - min_num
                allv = (allv / (meancount-2))

                print("{" + str(allv) + "}")
                lcd.draw_string(10, 30, "rawx:%0.1f"% (allv), lcd.YELLOW, lcd.BLACK)

            else:
                print("WAIT FULL")
        else:#如果是无效帧
            meanlost = meanlost + 1
            if meanlost > 10:
                meanfound = False

        #如果按了退出键
        if key_6.value() == 0:
            utime.sleep_ms(30)
            if key_6.value() == 0:
                while key_6.value() == 0:
                    pass
                break
        elif key_1.value() == 0:
            utime.sleep_ms(30)
            if key_1.value() == 0:
                #1键设置正150点校准
                x2cm_x150 = allv
                lcd.draw_string(10, 90, "x_upper150:%0.1f"%allv, lcd.YELLOW, lcd.BLACK)
        elif key_2.value() == 0:
            utime.sleep_ms(30)
            if key_2.value() == 0:
                #2键设置正75点校准
                x2cm_x75 = allv
                lcd.draw_string(10, 90, "x_upper75:%0.1f"%allv, lcd.YELLOW, lcd.BLACK)

        elif key_3.value() == 0:
            utime.sleep_ms(30)
            if key_3.value() == 0:
                #3键设置0点
                x2cm_x0 = allv
                lcd.draw_string(10, 60, "x0:%0.1f"%allv, lcd.YELLOW, lcd.BLACK)
        elif key_4.value() == 0:
            utime.sleep_ms(30)
            if key_4.value() == 0:
                #4键设置负75点
                x2cm_Fx75 = allv
                lcd.draw_string(10, 120, "x_lower75:%0.1f"%allv, lcd.YELLOW, lcd.BLACK)
        elif key_5.value() == 0:
            utime.sleep_ms(30)
            if key_5.value() == 0:
                #5键设置负150点
                x2cm_Fx150 = allv
                lcd.draw_string(10, 120, "x_lower150:%0.1f"%allv, lcd.YELLOW, lcd.BLACK)
        elif key_7.value() == 0:
            utime.sleep_ms(30)
            if key_7.value() == 0:
                #5键设置确认
                lcd.draw_string(10, 180, "JizoZhunOK", lcd.YELLOW, lcd.BLACK)

                x2cm_k_upper_150to75 = 75 / ((x2cm_x150 - x2cm_x75 ) + 0.000000001)
                #x2cm_a_upper_150to75 = -1 * x2cm_k_upper_150to75 * x0

                x2cm_k_upper_75to0 = 75 / ((x2cm_x75 - x2cm_x0)  + 0.000000001)
                #x2cm_a_upper_75to0 = -1 * x2cm_k_upper_75to0 * x0

                x2cm_k_lower_75to0 = 75 / ((x2cm_x0 - x2cm_Fx75) + 0.000000001)
                #x2cm_a_lower_75to0 = -1 * x2cm_k_lower_75to0 * x0

                x2cm_k_lower_150to75 = 75 / ((x2cm_Fx75 - x2cm_Fx150) + 0.000000001)
                #x2cm_k_lower_150to75 = -1 * x2cm_k_lower * x0


                #lcd.draw_string(10, 140, "k_u:%.6f;a_u:%.6f"%(x2cm_k_upper, x2cm_a_upper), lcd.YELLOW, lcd.BLACK)
                #lcd.draw_string(10, 160, "k_d:%.6f;a_down:%.6f"%(x2cm_k_lower, x2cm_a_lower), lcd.YELLOW, lcd.BLACK)




#def test2():

#def test3():


#显示液晶
lcd.clear()
lcd.draw_string(100, 100, "select test:", lcd.YELLOW, lcd.BLACK)

x2cm_k_upper_150to75 = 75 / (x2cm_x150 - x2cm_x75)
#x2cm_a_upper_150to75 = -1 * x2cm_k_upper_150to75 * x0

x2cm_k_upper_75to0 = 75 / (x2cm_x75 - x2cm_x0)
#x2cm_a_upper_75to0 = -1 * x2cm_k_upper_75to0 * x0

x2cm_k_lower_75to0 = 75 / (x2cm_x0 - x2cm_Fx75)
#x2cm_a_lower_75to0 = -1 * x2cm_k_lower_75to0 * x0

x2cm_k_lower_150to75 = 75 / (x2cm_Fx75 - x2cm_Fx150)
#x2cm_k_lower_150to75 = -1 * x2cm_k_lower * x0

while True:
    if key_1.value() == 0:
        utime.sleep_ms(30)
        if key_1.value() == 0:
            test2()
            lcd.clear()
            lcd.draw_string(100, 100, "select op:", lcd.YELLOW, lcd.BLACK)

    elif key_2.value() == 0:
        utime.sleep_ms(30)
        if key_2.value() == 0:
            test3()
            lcd.clear()
            lcd.draw_string(100, 100, "select op:", lcd.YELLOW, lcd.BLACK)

    elif key_3.value() == 0:
        utime.sleep_ms(30)
        if key_3.value() == 0:
            test4()
            lcd.clear()
            lcd.draw_string(100, 100, "select op:", lcd.YELLOW, lcd.BLACK)
    elif key_4.value() == 0:
        utime.sleep_ms(30)
        if key_4.value() == 0:
            jiaozhun()
    #time.sleep_ms(500)
mic.deinit()
