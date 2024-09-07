#! /usr/bin/env python3
# -*- coding: utf-8 -*-

env_name = 'H2017'
policy = "Gaussian"

eval = False
gamma = 0.99
tau = 0.005
lr = 0.00015
alpha = 0.2

automatic_entropy_tuning = True
seed = 123456

hidden_size = 64
updates_per_step =1 #1 time step에 Update를 몇 번 할것인지
target_update_interval = 1 #Update간의 간격을 나타냄 (ex. 3이면 Update를 요청 받았을 때 3의 배수번째로 요청하는 것만 업데이트함)
Current_Data_Selection_Ratio = 0.8
Clustering_K = 5
Is_Clearing_Memory = True
Success_Standard = 0.9

num_steps = 10000001
batch_size = 4096
start_steps = 30000
max_episode_steps = 128
time_sleep_interval = 0.00
action_weight = 20

isExit_IfSuccessLearning = False #목표 달성 시(success rate 0.9이상일 때) 학습을 종료할 것인지

replay_size = num_steps #1000000
cuda = "cuda"
