import numpy as np
import matplotlib.pyplot as plt



x = [1,2,3,4,5,6,7,8]
HP_step_avg = [39.5, 44, 36, 38.5, 37, 37, 36.5, 36.5]
HP_step_max = [43, 45, 43,45,41,45,43,43]
HP_step_min = [35,43,29,31,31,31,31,31]

SC_step_avg = [27,27,29.5,31.5,31.5,33,32.5,34.5]
SC_step_max = [27,27,35,33,37,37,37,39]
SC_step_min = [27,27,27,29,27,31,31,31]

PP_step_avg = [42,44.5,39.25,34,35,36,32,32]
PP_step_max = [45,45,45,45,41,45,37,37]
PP_step_min = [35,43,31,29,31,31,29,29]

fig = plt.figure(figsize=(6,3))


plt.figure('Fill', facecolor='lightgray')
plt.title('Fill', fontsize=20)
plt.xlabel('Flight Times', fontsize=14)
plt.ylabel('Movement Steps', fontsize=14)
plt.ylim((0, 60))
plt.tick_params(labelsize=10)
plt.grid(linestyle=':')


# plt.plot(x, HP_step_max, linewidth=0.1, c='white')
# plt.plot(x, HP_step_min, linewidth=0.1, c='white')
plt.plot(x, HP_step_avg, c='dodgerblue')

# plt.plot(x, SC_step_max, linewidth=0.1, c='white')
# plt.plot(x, SC_step_min, linewidth=0.1, c='white')
plt.plot(x, SC_step_avg, c='orangered')

# plt.plot(x, PP_step_max, linewidth=0.1, c='white')
# plt.plot(x, PP_step_min, linewidth=0.1, c='white')
plt.plot(x, PP_step_avg, c='limegreen')

# 填充
# plt.fill_between(x, HP_step_max,HP_step_min,  color='dodgerblue', alpha=0.3)
# plt.fill_between(x, SC_step_max,SC_step_min,  color='orangered', alpha=0.3)
# plt.fill_between(x, PP_step_max,PP_step_min,  color='limegreen', alpha=0.3)

plt.legend(loc = 3)
plt.show()
plt.show()