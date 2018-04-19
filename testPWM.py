import pigpio

pi = pigpio.pi()

# pins numbering
R = 3
P = 18
T = 19
Y = 13
ARM = 17

# DC frequency
f = 73.53

# set range of duty cycle from 0-100
pi.set_PWM_range(R,10000)
pi.set_PWM_range(P,10000)
pi.set_PWM_range(T,10000)
pi.set_PWM_range(Y,10000)
pi.set_PWM_range(ARM,10000)
# initialize frequency to 73.5 Hz
pi.set_PWM_frequency(R,f)
pi.set_PWM_frequency(P,f)
pi.set_PWM_frequency(T,f)
pi.set_PWM_frequency(Y,f)
pi.set_PWM_frequency(ARM,f)
# initialize duty cycles
pi.set_PWM_dutycycle(R,1200)
pi.set_PWM_dutycycle(P,1202)
pi.set_PWM_dutycycle(T,800) # max 1300
pi.set_PWM_dutycycle(Y,1204)
pi.set_PWM_dutycycle(ARM,1200)

