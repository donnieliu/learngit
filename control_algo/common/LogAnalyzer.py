import time

import matplotlib.pyplot as plt
import numpy as np
import os
import re
from decimal import Decimal

plt.rcParams["font.sans-serif"] = ["SimHei"]
plt.rcParams["font.family"] = "sans-serif"
plt.rcParams['axes.unicode_minus'] = False

file_name = "ControlAlgo.INFO"
l = 0

'''
KPI defined
'''

C0 = []
C1 = []
C2 = []
C3 = []
C03 = []
p_term = []
i_term = []
d_term = []
ff_result = []

ACC_STATE = []
ICC_STATE = []
LongAccelValid = []
LongAccelCmd = []

SteerTorqueValid = []
SteerTorqueCmd = []

TargetAngle = []
ActualAngle = []

AccControlMsgOnOff = []
AccControlMsgTimeGap = []

EgoSpeed = []
CipvValid = []
CipvSpeed = []
CipvDistance = []
SpeedError = []
DistanceError = []

EgoNextFlag = 0
EgoFlagCounter = 0

CipvNextFlag = 0
CipvFlagCounter = 0
for line in open(file_name, "r", encoding='UTF-8'):
    l = l + 1
    line = line.strip()  # 去掉每行头尾空白
    if not len(line) or line.startswith('#'):  # 判断是否是空行或注释行
        continue

    # "state_machine_msg_->acc_active"
    li = re.findall("state_machine_msg_->acc_active", line)
    if len(li) > 0:
        value = line.split()
        ACC_STATE.append(Decimal(value[-1]))

    # "state_machine_msg_->icc_active"
    li = re.findall("state_machine_msg_->icc_active", line)
    if len(li) > 0:
        value = line.split()
        ICC_STATE.append(Decimal(value[-1]))

    # "long_acceleration_valid"
    '''
    li = re.findall("long_acceleration_valid", line)
    if len(li) > 0:
        value = line.split()
        LongAccelValid.append(Decimal(value[-1]))
    '''
    # "long_acceleration_cmd"
    li = re.findall("long_acceleration_cmd", line)
    if len(li) > 0:
        value = line.split()
        LongAccelCmd.append(Decimal(value[-1]))

    # "steer_torque_valid"
    '''
    li = re.findall("steer_torque_valid", line)
    if len(li) > 0:
        value = line.split()
        SteerTorqueValid.append(Decimal(value[-1]))
    '''
    # "steer_torque_cmd"
    li = re.findall("steer_torque_cmd", line)
    if len(li) > 0:
        value = line.split()
        SteerTorqueCmd.append(Decimal(value[-1]))

    # "target_angle"
    li = re.findall("target_angle", line)
    if len(li) > 0:
        value = line.split()
        TargetAngle.append(Decimal(value[-1]))

    # "actual_angle"
    li = re.findall("actual_angle", line)
    if len(li) > 0:
        value = line.split()
        ActualAngle.append(Decimal(value[-1]))
    # "p_term"
    li = re.findall("p_term", line)
    if len(li) > 0:
        value = line.split()
        p_term.append(Decimal(value[-1]))

    # "i_term"
    li = re.findall("i_term", line)
    if len(li) > 0:
        value = line.split()
        i_term.append(Decimal(value[-1]))

    # "d_term"
    li = re.findall("d_term", line)
    if len(li) > 0:
        value = line.split()
        d_term.append(Decimal(value[-1]))

    # "ff_result"
    li = re.findall("ff_result", line)
    if len(li) > 0:
        value = line.split()
        ff_result.append(Decimal(value[-1]))

    # "acc_ctrl_msg_->on_off"
    li = re.findall("acc_ctrl_msg_->on_off", line)
    if len(li) > 0:
        value = line.split()
        AccControlMsgOnOff.append(Decimal(value[-1]))

    # "c0 -- c3"
    li = re.findall("c0 -- c3", line)
    if len(li) > 0:
        value = line.split()
        print("C3-C0", Decimal(value[-1]), Decimal(value[-3]),\
              Decimal(value[-5]), Decimal(value[-7]))
        # for v in value:
        C3.append(Decimal(value[-1]))
        C2.append(Decimal(value[-3]))
        C1.append(Decimal(value[-5]))
        C0.append(Decimal(value[-7]))
    # "acc_ctrl_msg_->timegap_toggle"

    li = re.findall("acc_ctrl_msg_->timegap_toggle", line)
    if len(li) > 0:
        value = line.split()
        AccControlMsgTimeGap.append(Decimal(value[-1]))

    # ">>>ego info"
    if EgoNextFlag == 1 and EgoFlagCounter == 2:
        value = line.split()
        EgoSpeed.append(Decimal(value[-1]))
        EgoNextFlag = 0
        EgoFlagCounter = 0
    elif EgoNextFlag == 1:
        li = re.findall(">>>ego info", line)
        if len(li) > 0:
            EgoFlagCounter = EgoFlagCounter + 1
    else:
        li = re.findall(">>>ego info", line)
        if len(li) > 0 and EgoNextFlag == 0:
            EgoNextFlag = 1
            EgoFlagCounter = EgoFlagCounter + 1

    # ">>>cipv info"
    if CipvNextFlag and CipvFlagCounter == 2:
        value = line.split()
        CipvSpeed.append(Decimal(value[-1]))
        CipvNextFlag = 0
        CipvFlagCounter = 0
    elif CipvNextFlag == 1:
        CipvFlagCounter = CipvFlagCounter + 1
    else:
        li = re.findall(">>>cipv info", line)
        if len(li) > 0 and CipvNextFlag == 0:
            CipvNextFlag = 1
            CipvFlagCounter = CipvFlagCounter + 1


def polynomial_coefficients(xs, coeffs):
    """ Returns a list of function outputs (`ys`) for a polynomial with the given coefficients and
    a list of input values (`xs`).

    The coefficients must go in order from a0 to an, and all must be included, even if the value is 0.
    """
    order = len(coeffs)
    #print(f'# This is C0: {coeffs[0]}.')

    ys = np.zeros(len(xs))  # Initialise an array of zeros of the required length.
    for i in range(order):
        ys += float(coeffs[i]) * xs ** i
    return ys


print("Number of points： ", l)
#fig = plt.figure(figsize=(30, 15))
fig = plt.figure(figsize=(200, 15))
ax1 = plt.subplot(4, 1, 1)
ax2 = plt.subplot(4, 1, 2)
ax3 = plt.subplot(4, 1, 3)
ax4 = plt.subplot(4, 1, 4)

#ax1.plot(AccControlMsgOnOff, 'b-', label='AccControlMsgOnOff')
#ax1.plot(AccControlMsgTimeGap, 'g--', label='AccControlMsgTimeGap')

# ax1.plot(SteerTorqueValid, 'c:', label='SteerTorqueValid')
# ax1.plot(LongAccelCmd, 'm-', label='LongAccelCmd')
ax1.plot(TargetAngle, 'm-', label='TargetAngle')
ax1.plot(ActualAngle, 'b-', label='ActualAngle')
# ax1.plot(LongAccelValid, 'y--', label='LongAccelValid')
ax1.set_ylabel('Debug Signals')
ax1.legend(loc='lower left')
ax1.set_title("BYD Log 分析", fontsize=20)
ax1.grid(True)

ax2.plot(ACC_STATE, 'r--', label='ACC_STATE')
ax2.plot(ICC_STATE, 'g-', label='ICC_STATE')
ax2.legend(loc='lower left')
ax2.set_ylabel("State Machines")
ax2.grid(True)

ax3.plot(SteerTorqueCmd, 'r-.', label='SteerTorqueCmd')
ax3.set_ylabel("SteerTorqueCmd")
ax3.plot(p_term, 'm-', label='p_term')
ax3.plot(i_term, 'b-', label='i_term')
ax3.plot(d_term, 'k-', label='d_term')
ax3.plot(ff_result, 'y-', label='ff_result')
ax3.legend(loc='lower left')
ax3.grid(True)

ax4.plot(C0, 'r--', label='CO', linewidth=2)
ax4.legend(loc='lower left')
ax4.set_ylabel("Camera C0")
ax4.grid(True)

'''
xs = np.linspace(0, 29, 60)
ax3.set_ylabel("Road shape")
#plt.ion()
for i in range(len(C0)):
    if i < 4000:
        continue
    print("Doing: ", i)
    ax3.cla()
    coeffs = [C0[i], C1[i], C2[i], C3[i]]
    ys = polynomial_coefficients(xs, coeffs)
    ax3.plot(xs, ys, 'r--', label='Coffs road', linewidth=2)
    coeffs = [0, C1[i], C2[i], C3[i]]
    ys = polynomial_coefficients(xs, coeffs)
    ax3.plot(xs, ys, 'g--', label='Center road', linewidth=3)
    coeffs = [-1.7, C1[i], C2[i], C3[i]]
    ys = polynomial_coefficients(xs, coeffs)
    ax3.plot(xs, ys, 'g-', label='Left road', linewidth=3)
    coeffs = [1.7, C1[i], C2[i], C3[i]]
    ys = polynomial_coefficients(xs, coeffs)
    ax3.plot(xs, ys, 'g-', label='Right road', linewidth=3)
    # fig.canvas.draw()
    # renderer = fig.canvas.renderer
    # ax3.draw(renderer)
    ax3.legend(loc='lower left')
    ax3.set_ylabel("Road shape")
'''
plt.xlabel("Signal order")
plt.ylabel("Speed Signals")
plt.show()
