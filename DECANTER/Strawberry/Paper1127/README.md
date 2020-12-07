<!--
 * @Author: your name
 * @Date: 2020-12-04 17:33:01
 * @LastEditTime: 2020-12-04 17:33:01
 * @LastEditors: your name
 * @Description: In User Settings Edit
 * @FilePath: \HEYTEAc:\Users\philt\Documents\GitHub\ICEWINE\DECANTER\Strawberry\Paper1127\README.md
-->

# Paper1127

MoCap based demonstration platform v0.0

Haopeng Hu

## Data

## Scripts

## Introduction

### Raw data 

- OptitrackData is used for data storation.
- No filter for noise attenuation. We assume the data is good enough.
- There must be NaNs in the raw data.

### Data compensation

- Gaussian process is used to cook the raw data.
- Interpolation methods such as spline/slerp is used for comparison.
- Kalman filter can also be used for comparison.

### Policy learning and trajectory generation

- GMM is used for motion generation.
- DMP can also be used for motion generation.
- Is it possible to develop a SEDS-like dynamic method?

