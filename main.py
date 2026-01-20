from motor import *
from mdv2 import *
from drivebase import *
from mpu6050 import MPU6050
from angle_sensor import AngleSensor
from pins import *
from abutton import *

# Mô tả hàm này...
async def test_di_chuyen__5Bs_5D(huong, so_lan):
  global thoi_gian, doan, list_goc_do_duoc
  await init_test(huong)
  await di_chuyen__5Bs_5D(huong, 0.1, so_lan)
  await di_chuyen__5Bs_5D(huong, 0.5, so_lan)
  await di_chuyen__5Bs_5D(huong, 1, so_lan)
  await di_chuyen__5Bs_5D(huong, 3, so_lan)

# Mô tả hàm này...
async def init_test(huong):
  global so_lan, thoi_gian, doan, list_goc_do_duoc
  robot.pid(Kp=0, Ki=0, Kd=0)
  if huong == 4:
    robot.speed(36, min_speed=36)
    robot.speed_ratio(front_left=1.6, front_right=2.6, rear_left=1.25, rear_right=1.25)
    robot.set_stop_coefficient(value=0.85)
  elif huong == 6:
    robot.speed(36, min_speed=36)
    robot.speed_ratio(front_left=1.6, front_right=2.6, rear_left=1.25, rear_right=1.25)
    robot.set_stop_coefficient(value=0.85)
  elif huong == 8 or huong == 2:
    robot.speed(4, min_speed=4)
    robot.speed_ratio(front_left=1, front_right=1.2, rear_left=1.25, rear_right=1.45)
    robot.set_stop_coefficient(value=8)
  elif huong == 7:
    pass
  elif huong == 9:
    pass

async def on_abutton_BOOT_pressed():
  global doan, list_goc_do_duoc, so_lan, thoi_gian, huong, x
  await asleep_ms(2000)
  await init_test(8)
  await test_di_chuyen__5Bs_5D(8, 3)

# Mô tả hàm này...
# Huong di chuyen tinh theo ban phim tren may tinh windows:
# 7 8 9 -> 8: di toi, 7: quay trai, 9: quay phai
# 4    6 -> 4: di ngang trai, 6: di ngang phai
#    2    -> 2: di lui
async def di_chuyen__5Bs_5D(huong, thoi_gian, so_lan):
  global doan, list_goc_do_duoc
  print('huong' + ': ' + str(huong))
  print('thoi gian' + ': ' + str(thoi_gian))
  if huong == 8:
    for count in range(int(so_lan)):
      await robot.forward_for(thoi_gian, unit=SECOND, then=BRAKE)
      await asleep_ms(1000)
  elif huong == 2:
    for count2 in range(int(so_lan)):
      await robot.backward_for(thoi_gian, unit=SECOND, then=BRAKE)
      await asleep_ms(1000)
  elif huong == 7:
    for count3 in range(int(so_lan)):
      await robot.turn_left_for(thoi_gian, unit=SECOND, then=BRAKE)
      await asleep_ms(1000)
  elif huong == 9:
    for count4 in range(int(so_lan)):
      await robot.turn_right_for(thoi_gian, unit=SECOND, then=BRAKE)
      await asleep_ms(1000)
  elif huong == 4:
    for count5 in range(int(so_lan)):
      await robot.move_left_for(thoi_gian, unit=SECOND, then=BRAKE)
      await asleep_ms(1000)
  elif huong == 6:
    for count6 in range(int(so_lan)):
      await robot.move_right_for(thoi_gian, unit=SECOND, then=BRAKE)
      await asleep_ms(1000)
  else:
    robot.stop()

# Mô tả hàm này...
async def test_di_chuyen__5Bcm_5D(huong, so_lan):
  global thoi_gian, doan, list_goc_do_duoc
  await init_test(huong)
  await di_chuyen__5Bcm_do_5D(huong, 15, so_lan)
  await di_chuyen__5Bcm_do_5D(huong, 30, so_lan)
  await di_chuyen__5Bcm_do_5D(huong, 60, so_lan)
  await di_chuyen__5Bcm_do_5D(huong, 100, so_lan)

# Mô tả hàm này...
# Huong di chuyen tinh theo ban phim tren may tinh windows:
# 7 8 9 -> 8: di toi, 7: quay trai, 9: quay phai
# 4    6 -> 4: di ngang trai, 6: di ngang phai
#    2    -> 2: di lui
async def di_chuyen__5Bcm_do_5D(huong, doan, so_lan):
  global thoi_gian, list_goc_do_duoc
  print('huong' + ': ' + str(huong))
  print('doan' + ': ' + str(doan))
  if huong == 8:
    for count7 in range(int(so_lan)):
      await robot.forward_for(doan, unit=CM, then=BRAKE)
      await asleep_ms(1000)
  elif huong == 2:
    for count8 in range(int(so_lan)):
      await robot.backward_for(doan, unit=CM, then=BRAKE)
      await asleep_ms(1000)
  elif huong == 7:
    for count9 in range(int(so_lan)):
      await robot.turn_left_for(doan, unit=DEGREE, then=BRAKE)
      list_goc_do_duoc.append(angle_sensor.heading)
      await asleep_ms(1000)
  elif huong == 9:
    for count10 in range(int(so_lan)):
      await robot.turn_right_for(doan, unit=DEGREE, then=BRAKE)
      list_goc_do_duoc.append(angle_sensor.heading)
      await asleep_ms(1000)
  elif huong == 4:
    for count11 in range(int(so_lan)):
      await robot.move_left_for(doan, unit=CM, then=BRAKE)
      await asleep_ms(1000)
  elif huong == 6:
    for count12 in range(int(so_lan)):
      await robot.move_right_for(doan, unit=CM, then=BRAKE)
      await asleep_ms(1000)
  else:
    robot.stop()

# Mô tả hàm này...
async def tim_dead_band():
  global huong, so_lan, thoi_gian, doan, list_goc_do_duoc
  doan = 10
  for count13 in range(20):
    print('sp' + ': ' + str(doan))
    motor2.run(doan)
    await asleep_ms(3000)
    doan = (doan if isinstance(doan, (int, float)) else 0) + 2
    motor2.run(0)
    await asleep_ms(3000)

# Mô tả hàm này...
async def test_quay_goc__5Bdo_5D(huong, so_lan):
  global thoi_gian, doan, list_goc_do_duoc
  await init_test(huong)
  list_goc_do_duoc = []
  await di_chuyen__5Bcm_do_5D(huong, 15, so_lan)
  await di_chuyen__5Bcm_do_5D(huong, 30, so_lan)
  await di_chuyen__5Bcm_do_5D(huong, 60, so_lan)
  await di_chuyen__5Bcm_do_5D(huong, 90, so_lan)
  await di_chuyen__5Bcm_do_5D(huong, 135, so_lan)
  await di_chuyen__5Bcm_do_5D(huong, 180, so_lan)
  print(list_goc_do_duoc)

doan = None
list_goc_do_duoc = None
so_lan = None
thoi_gian = None
huong = None
x = None
md_v2 = MotorDriverV2()
motor1 = DCMotor(md_v2, E2, reversed=True)
motor2 = DCMotor(md_v2, E1, reversed=True)
motor3 = DCMotor(md_v2, M2, reversed=False)
motor4 = DCMotor(md_v2, M1, reversed=True)
robot = DriveBase(MODE_MECANUM, m1=motor1, m2=motor2, m3=motor3, m4=motor4)
imu = MPU6050()
angle_sensor = AngleSensor(imu)
led_D13 = Pins(D13_PIN)
btn_BOOT= aButton(BOOT_PIN)

def deinit():
  robot.stop()
  btn_BOOT.deinit()

import yolo_uno
yolo_uno.deinit = deinit

async def task_w_F_d_x():
  global doan, list_goc_do_duoc, so_lan, thoi_gian, huong, x
  while True:
    await asleep_ms(1000)
    led_D13.toggle()

async def setup():
  global doan, list_goc_do_duoc, so_lan, thoi_gian, huong, x
  print('App started')
  neopix.show(0, hex_to_rgb('#ff0000'))
  motor1.set_encoder(rpm=250, ppr=6, gears=110)
  motor2.set_encoder(rpm=250, ppr=6, gears=110)
  robot.size(wheel=60, width=200)
  angle_sensor.calibrate(1000)
  create_task(angle_sensor.run())
  robot.angle_sensor(angle_sensor)
  robot.use_gyro(False)
  neopix.show(0, hex_to_rgb('#00ff00'))
  list_goc_do_duoc = []
  print((md_v2.battery()))

  btn_BOOT.pressed(on_abutton_BOOT_pressed)
  create_task(task_w_F_d_x())

async def main():
  await setup()
  while True:
    await asleep_ms(100)

run_loop(main())
