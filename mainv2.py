import machine
from machine import Pin
from machine import ADC
import time

fuktsensor = ADC(26) #sensor, resistiv
fuktmin = 2000 # Nedre fuktmåler før pumpe vil skrus på.
fuktmax = 5000 # Øvre fuktmåler slik at pompen vil skru seg av uanhengig av den faste tiden som er satt.

vannsensor = ADC(27) #sensor, resistiv
tanktomverdi = 8000 # Verdi som skiller mellom en tank med og uten vann.
vannteller = 0 #teller for vannmåler, gjør slik at hvis tank er tom for vann vil det kun logges en gang.

pumpe = Pin(15, Pin.OUT) #motorstyring, pin 15 og 14 avhengig av retning.
pumpetid = 10 #antall sekunder pumpe kjører når den aktiveres.
pumpeteller = 0 #teller for timing av pumpe.

alarmteller = 0 #teller for å forsinke print og logging av varsel på temperatur.

#initialiserer kode, krever input av dato, dette vil gi tidspunkt for data som blir logget.
print()
print(" YYYY MM DD HH MM SS")
dateTime = (input ("Enter current date & time: "))+' 0 0'
synchronisedTime = time.mktime(list(map(int, tuple(dateTime.split(' ')))))
timeDelta = synchronisedTime - int(time.time())
print(" plantevanner er igang")
 
# Definerer i2c grensesnitt, GPIO pin 4 og 1, 3.3v og gnd.
sda = machine.Pin(4)  #gpio4
scl = machine.Pin(1)  #gpio1

i2c = machine.I2C(0, sda=sda, scl=scl, freq=100000)
i2c_addr = 0x77

#sjekker at bme280 responderer over i2c.
print('i2c devices found at')
devices = i2c.scan()
if devices:
    for i in devices:
        print(hex(i))
print()

# Setter opp funksjoner for plantevanning.

def timeNow():
    return time.localtime(time.time() + timeDelta)

def pumpe_on():
    pumpe.value(1)
    print("Vanner, pumpe skrus på.")
    
def pumpe_off():
    pumpe.value(0)
    print("Planten har nok vann.")
        
while True:
    
    # Kode under initialiserer og omregner verdier fra bme280 sensor.
    
    # All setup parameters are defined in the datasheet
    # humidity  oversampling
    i2c.writeto_mem(i2c_addr, 0xf2, b'\x03')  # ctrl_hum 00000 011
    time.sleep_ms(100)

    # temp oversampling / pressure / oversampling / sensor mode
    i2c.writeto_mem(i2c_addr, 0xf4, b'\x6F')  # ctrl_meas 011 011 11

    # venter på write, sensor krasjer om ikke delay er satt her.
    time.sleep(0.1)

    # The sensor presents as a memory mapped device on teh i2c bus
    reg_base_addr = 0x88  # memory map is from 0x88 to 0xff
    block_size = 0x100 - reg_base_addr  # bytes to read
    r = i2c.readfrom_mem(i2c_addr, reg_base_addr, block_size)  # Read the sensor into a buffer


    # read two bytes as an unsigned int
    def read_const_u(a):
        return r[a - reg_base_addr] + (r[a - reg_base_addr + 1] << 8)


    # read two bytes as a signed int
    def read_const_s(a):
        v = r[a - reg_base_addr] + (r[a - reg_base_addr + 1] << 8)
        if v > 32767:
            v = v - 65536
        return v


    # TEMPERATURE

    # get raw temperature
    temp_msb = r[0xfa - reg_base_addr]
    temp_lsb = r[0xfb - reg_base_addr]
    temp_xlsb = r[0xfc - reg_base_addr]
    adc_t = (temp_msb << 12) + (temp_lsb << 4) + (temp_xlsb >> 4)

    # get compensation parameters (These are constants, only read once in a production system)
    dig_t1 = read_const_u(0x88)
    dig_t2 = read_const_s(0x8a)
    dig_t3 = read_const_s(0x8c)

    # calc temperature
    # Shown as done in the datasheet
    var1 = ((adc_t >> 3) - (dig_t1 << 1)) * (dig_t2 >> 11)
    var2 = (((((adc_t >> 4) - dig_t1) * ((adc_t >> 4) - dig_t1)) >> 12) * dig_t3) >> 14
    t_fine = (var1 + var2)
    t = (t_fine * 5 + 128) >> 8

    # PRESSURE
    # get raw pressure
    press_msb = r[0xf7 - reg_base_addr]
    press_lsb = r[0xf8 - reg_base_addr]
    press_xlsb = r[0xf9 - reg_base_addr]
    adc_p = (press_msb << 12) + (press_lsb << 4) + (press_xlsb >> 4)

    # get compensation parameters (These are constants, only read once in a production system)

    dig_p1 = read_const_u(0x8e)
    dig_p2 = read_const_s(0x90)
    dig_p3 = read_const_s(0x92)
    dig_p4 = read_const_s(0x94)
    dig_p5 = read_const_s(0x96)
    dig_p6 = read_const_s(0x98)
    dig_p7 = read_const_s(0x9a)
    dig_p8 = read_const_s(0x9c)
    dig_p9 = read_const_s(0x9e)

    # calc pressure
    # Shown as done in the datasheet
    var1 = t_fine - 128000
    var2 = var1 * var1 * dig_p6
    var2 = var2 + ((var1 * dig_p5) << 17)
    var2 = var2 + (dig_p4 << 35)
    var1 = ((var1 * var1 * dig_p3) >> 8) + ((var1 * dig_p2) << 12)
    var1 = ((1 << 47) + var1) * dig_p1 >> 33
    if var1 == 0:  # divide by zero check
        p = 0
    else:
        p = 1048576 - adc_p
        p = int((((p << 31) - var2) * 3125) / var1)
        var1 = (dig_p9 * (p >> 13) * (p >> 13)) >> 25
        var2 = (dig_p8 * p) >> 19
        p = ((p + var1 + var2) >> 8) + (dig_p7 << 4)

    # HUMIDITY

    # get raw pressure
    hum_msb = r[0xfd - reg_base_addr]
    hum_lsb = r[0xfe - reg_base_addr]
    adc_h = (hum_msb << 8) + hum_lsb

    # get compensation parameters (These are constants, only read once in a production system)

    dig_h1 = r[0xa1 - reg_base_addr]
    dig_h2 = read_const_s(0xe1)
    dig_h3 = r[0xe3 - reg_base_addr]
    dig_h4 = (r[0xe4 - reg_base_addr] << 4) + (r[0xe5 - reg_base_addr] & 0x0f)
    dig_h5 = (r[0xe6 - reg_base_addr] << 4) + ((r[0xe5 - reg_base_addr] & 0x00f0) >> 4)
    dig_h6 = r[0xe7 - reg_base_addr]

    # calc humidity
    # Shown as done in the datasheet
    v_x1_u32r = t_fine - 76800

    v_x1_u32r = (((((adc_h << 14) - (dig_h4 << 20) - (dig_h5 * v_x1_u32r)) + 16384) >> 15) * (((((((v_x1_u32r * (
        dig_h6)) >> 10) * (((v_x1_u32r * dig_h3) >> 11) + 32768)) >> 10) + 2097152) * dig_h2 + 8192) >> 14))

    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * dig_h1) >> 4))

    # limit checks
    if v_x1_u32r < 0:
        v_x1_u32r = 0

    if v_x1_u32r > 0x19000000:
        v_x1_u32r = 0x19000000

    h = v_x1_u32r >> 12
    
    
    
    
# Kodestart for plantevanning, bme verdier utregnet.

    fukt = fuktsensor.read_u16()
    print("fukt: ", fukt)
    dateTime = timeNow()
    print("{:02d}-{:02d}-{:04d} {:02d}:{:02d}:{:02d}".format(dateTime[2],dateTime[1],dateTime[0],dateTime[3],dateTime[4],dateTime[5]))
    print("Temperature is " + str(t / 100) + " degrees")
    print("Pressure is " + str(p / 25600) + " mbar")
    print("Humidity is " + str(h / 1024) + " %")
    time.sleep(1) #setter forsinkelse i koden slik at print til shell blir oversiktig, denne forsinkelsen vil avgjøre tiden for diverse sykluser som er basert på antall iterasjoner.
    print()
    print()
    


# Printer varsel for "min" og "max" grense for temperatur
    if (t / 100) > 26: 
        print("Det er skummelt varmt for den lille planten nå.")
        alarmteller = alarmteller + 1
        
        # Alarmlogg ved for høy temperatur
        # Bruker en teller slik at det ikke logges hvert sekund
        if alarmteller == 1:
            
            with open("LoggVanningssytem.txt", mode="a") as f:
                f.write('--ALARM--FOR HØY TEMPERATUR--\n')
                f.write('Dato og klokkeslett: ' + str("{:02d}-{:02d}-{:04d} {:02d}:{:02d}:{:02d}".format(dateTime[2],dateTime[1],dateTime[0],dateTime[3],dateTime[4],dateTime[5])) + '\n')
                f.write('Temperatur: ' + str((t / 100)) + ' °C.\n')
                f.write('\n')
                
        # Etter 10 minutter resettes alarmtelleren
        if alarmteller == 10:
            alarmteller = 0

    elif (t / 100) < 24: 
        print("Det er for kaldt for planten nå!")
        alarmteller = alarmteller + 1
        
        # Alarmlogg ved for lav temperatur
        # Bruker en teller slik at det ikke logges hvert sekund
        if alarmteller == 1:
            with open("LoggVanningssytem.txt", mode="a") as f:
                f.write('--ALARM--FOR LAV TEMPERATUR--\n')
                f.write('Dato og klokkeslett: ' + str("{:02d}-{:02d}-{:04d} {:02d}:{:02d}:{:02d}".format(dateTime[2],dateTime[1],dateTime[0],dateTime[3],dateTime[4],dateTime[5])) + '\n')
                f.write('Temperatur: ' + str((t / 100)) + ' °C.\n')
                f.write('\n')
        # Etter 10 minutter resettes alarmtelleren
        if alarmteller == 10:
            alarmteller = 0


#Vannsensor som dekterer om tanken er tom eller ikke.
    vann = vannsensor.read_u16()
    if vann < tanktomverdi or vannteller != 0:
        vannteller = vannteller + 1

        if vannteller == 1:
            with open("LoggVanningssytem.txt", mode="a") as f:
                f.write('Vanntank er tom.\n')
                f.write('Dato og klokkeslett: ' + str("{:02d}-{:02d}-{:04d} {:02d}:{:02d}:{:02d}".format(dateTime[2],dateTime[1],dateTime[0],dateTime[3],dateTime[4],dateTime[5])) + '\n')
                f.write('Vannsensor: ' + str(vann) + '\n')
                f.write('\n')
                
    if vann >= tanktomverdi:
        vannteller = 0
        print("Vanntanken har vann.")
        
    else:
        print("Vanntanken er tom.")



#Fuktsensor som velger om pumpe skal på eller av.
    if fukt < fuktmin  and vann >= tanktomverdi or pumpeteller != 0: #pumpe vil gå om kondisjonene er møtt. pumpeteller holder pumpe gående i 10 iterasjoner av koden.
        pumpeteller = pumpeteller + 1
        
        if fukt > fuktmax:
            pumpe_off()
            pumpeteller = 0
        else:
            pumpe_on()

            #Logging av data når pumpen starter
            if pumpeteller == 1: # Bruker telleren slik at det bare logges en gang når pumpen går på.
                with open("LoggVanningssytem.txt", mode="a") as f:
                    f.write('Pumpe aktivert \n')
                    f.write('Dato og klokkeslett: ' + str("{:02d}-{:02d}-{:04d} {:02d}:{:02d}:{:02d}".format(dateTime[2],dateTime[1],dateTime[0],dateTime[3],dateTime[4],dateTime[5])) + '\n')
                    f.write('Fukt: ' + str(fukt) + '\n')
                    f.write('Temperatur: ' + str(t/100) + ' °C' +'\n')
                    f.write('\n')
        
    else:
        pumpe_off()
    
    if pumpeteller == pumpetid:
        pumpeteller = 0

    print("teller: " + str(pumpeteller)) #Forteller hvor lenge det er igjen av gjeldende pumpesyklus.