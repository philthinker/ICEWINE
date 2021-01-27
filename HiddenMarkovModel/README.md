<!--
 * @Author: Haopeng Hu
 * @Date: 2020-12-30 10:02:10
 * @LastEditTime: 2021-01-27 14:42:24
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: \undefinedc:\Users\philt\Documents\GitHub\ICEWINE\HiddenMarkovModel\README.md
-->

# HiddenMarkovModel

Haopeng Hu

2020.12.30

All rights reserved.

## Classes

- HMMZero: Zero version of typical hidden markov model.
- HSMMZero: Zero version of typical hidden semi-markov model.
- TrajHSMMZero < HSMMZero: Zero version of traj-HSMM.
- LfDHSMMZero < TrajHSMMZero: Zero version of HSMM used for LfD. It shares the same demo with @TrajHSMMZero.
- LfDHSMMOne < HSMMZero: The HSMM of LfD usage. It enjoys the functions in unit quaternion space.
- TPTrajHSMMZero < TrajHSMMZero: Task-parameterized Traj-HSMM.

## Demos

- demoHMM0: Demo for the usage of @HMMZero.
- demoHSMM0: Demo for using @HSMMZero to emulate HSMM.
- demoTrajHSMM0: Demo for using @TrajHSMMZero.
- demoTPHSMM0: Demo for using @TPTrajHSMMZero.