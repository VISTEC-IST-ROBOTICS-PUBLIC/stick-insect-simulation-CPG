#python

# ********************************************************* #
# ********************************************************* #
#                   Developer information                   #
# ********************************************************* #
# ********************************************************* #

# Developed by:
# Thirawat Chuthong (Joe) 
# thirawat.c_s21@vistec.ac.th
# Bio-inspired Robotics and Neural Engineering Laboratory,
# School of Information Science and Technology,
# Vidyasirimedhi Institute of Science and Technology, 
# Wangchan Valley 555 Moo 1, Payupnai, Wangchan, 21210 Rayong, Thailand

# Citation:
# Paper title: Self-Organized Stick Insect-Like Locomotion under Decentralized Adaptive Neural Control: From Biological Investigation to Robot Simulation
# Link: https://onlinelibrary.wiley.com/doi/10.1002/adts.202300228

# @article{alexander2023_selfOrg,
#     author = {Larsen, Alexander and Büscher, Thies and Chuthong, Thirawat and Pairam, Thipawan and Bethge, Hendrik and Gorb, Stanislav and Manoonpong, Poramate},
#     year = {2023},
#     month = {06},
#     pages = {},
#     title = {Self‐Organized Stick Insect‐Like Locomotion under Decentralized Adaptive Neural Control: From Biological Investigation to Robot Simulation},
#     volume = {6},
#     journal = {Advanced Theory and Simulations},
#     doi = {10.1002/adts.202300228 }
# }

# ************************************************************************************************* #
# ************************************************************************************************* #


import math
import numpy as np
# ********************************************************* #
# ********************************************************* #
#                        Initial system                     #
# ********************************************************* #
# ********************************************************* #
def sysCall_init():
    sim = require('sim')

    # If False --> using RBFN
    self.manual_gen = True
    self.time_lap = 30 # second
    
    self.FL_joints = []
    self.FR_joints = []
    self.ML_joints = []
    self.MR_joints = []
    self.HL_joints = []
    self.HR_joints = []

    self.x_distance = 0
    self.y_distance = 0
    self.z_distance = 0
    self.final_x_distance = 0
    
    jointName = ['/TR0','/CR0','/FR0',
                 '/TR1','/CR1','/FR1',
                 '/TR2','/CR2','/FR2',
                 '/TL0','/CL0','/FL0',
                 '/TL1','/CL1','/FL1',
                 '/TL2','/CL2','/FL2']
    legName = ['R0','R1','R2','L0','L1','L2']
    legpartName = ['COXA','FEMUR','TIBIA']
    bodypartName = ['abdomen', 'thorax_hind','thorax_front','caput']

    footLeft_name = ['/FOOT_L0', '/FOOT_L1', '/FOOT_L2']
    footRight_name = ['/FOOT_R0', '/FOOT_R1', '/FOOT_R2']
    forceSensor_name = ['/L0_fs', '/L1_fs', '/L2_fs', '/R0_fs', '/R1_fs', '/R2_fs']

    
    # ***************************** #
    #             Graph             #
    # ***************************** #

    # ******************* CPG graph ******************** # 
    self.cpg_graph = sim.getObject('/cpg_graph')
    self.cpg_val1 = sim.addGraphStream(self.cpg_graph, 'val1', '', 0, [1, 0, 0])
    self.cpg_val2 = sim.addGraphStream(self.cpg_graph, 'val2', '', 0, [0, 1, 0])
    self.cpg_val3 = sim.addGraphStream(self.cpg_graph, 'val3', '', 0, [1, 0.5, 0])
    self.cpg_val4 = sim.addGraphStream(self.cpg_graph, 'val4', '', 0, [0, 0.5, 1])
    self.cpg_val5 = sim.addGraphStream(self.cpg_graph, 'val5', '', 0, [1, 0, 1])
    self.cpg_val6 = sim.addGraphStream(self.cpg_graph, 'val6', '', 0, [0.5, 0, 0.5])

    # ***************** Trajectories ******************* # 
    # options setting
    # 0 - show all
    # 1 - stream is not show
    # 2 - label is not show
    # 4 - scatter plot
    options = 2
    dim = 3
    lineWidth = 2
    counting = 0.2

    self.traj_FL = sim.getObject('/traj_FL_graph')
    self.traj_FL_x = sim.addGraphStream(self.traj_FL, 'x', 'm', options, [1,0,0])
    self.traj_FL_y = sim.addGraphStream(self.traj_FL, 'y', 'm', options, [1,0,0])
    self.traj_FL_z = sim.addGraphStream(self.traj_FL, 'z', 'm', 1,       [1,0,0])
    sim.addGraphCurve( self.traj_FL, 'graph name', dim, [self.traj_FL_x, self.traj_FL_y, self.traj_FL_z], [0,0,0], 'm', options, [0.2,0.2,0.2], lineWidth )
    
    self.traj_ML = sim.getObject('/traj_ML_graph')
    self.traj_ML_x = sim.addGraphStream(self.traj_ML, 'x', 'm', options, [0,1,0])
    self.traj_ML_y = sim.addGraphStream(self.traj_ML, 'y', 'm', options, [0,1,0])
    self.traj_ML_z = sim.addGraphStream(self.traj_ML, 'z', 'm', 1,       [0,1,0])
    sim.addGraphCurve( self.traj_ML, 'graph name', dim, [self.traj_ML_x, self.traj_ML_y, self.traj_ML_z], [0,0,0], 'm', options, [0,0.5,0], lineWidth )

    self.traj_HL = sim.getObject('/traj_HL_graph')
    self.traj_HL_x = sim.addGraphStream(self.traj_HL, 'x', 'm', options, [0,0,1])
    self.traj_HL_y = sim.addGraphStream(self.traj_HL, 'y', 'm', options, [0,0,1])
    self.traj_HL_z = sim.addGraphStream(self.traj_HL, 'z', 'm', 1,       [0,0,1])
    sim.addGraphCurve( self.traj_HL, 'graph name', dim, [self.traj_HL_x, self.traj_HL_y, self.traj_HL_z], [0,0,0], 'm', options, [1,0,0], lineWidth )

    self.traj_FR = sim.getObject('/traj_FR_graph')
    self.traj_FR_x = sim.addGraphStream(self.traj_FR, 'x', 'm', options, [1,0,0])
    self.traj_FR_y = sim.addGraphStream(self.traj_FR, 'y', 'm', options, [1,0,0])
    self.traj_FR_z = sim.addGraphStream(self.traj_FR, 'z', 'm', 1,       [1,0,0])
    sim.addGraphCurve( self.traj_FR, 'graph name', dim, [self.traj_FR_x, self.traj_FR_y, self.traj_FR_z], [0,0,0], 'm', options, [0.2,0.2,0.2], lineWidth )

    self.traj_MR = sim.getObject('/traj_MR_graph')
    self.traj_MR_x = sim.addGraphStream(self.traj_MR, 'x', 'm', options, [0,1,0])
    self.traj_MR_y = sim.addGraphStream(self.traj_MR, 'y', 'm', options, [0,1,0])
    self.traj_MR_z = sim.addGraphStream(self.traj_MR, 'z', 'm', 1,       [0,1,0])
    sim.addGraphCurve( self.traj_MR, 'graph name', dim, [self.traj_MR_x, self.traj_MR_y, self.traj_MR_z], [0,0,0], 'm', options, [0,0.5,0], lineWidth )

    self.traj_HR = sim.getObject('/traj_HR_graph')
    self.traj_HR_x = sim.addGraphStream(self.traj_HR, 'x', 'm', options, [0,0,1])
    self.traj_HR_y = sim.addGraphStream(self.traj_HR, 'y', 'm', options, [0,0,1])
    self.traj_HR_z = sim.addGraphStream(self.traj_HR, 'z', 'm', 1,       [0,0,1])
    sim.addGraphCurve( self.traj_HR, 'graph name', dim, [self.traj_HR_x, self.traj_HR_y, self.traj_HR_z], [0,0,0], 'm', options, [1,0,0], lineWidth )




    # ***************************** #
    #             Object            #
    # ***************************** #
    self.IMU = sim.getObject('/IMU')
    for i in range(3):
        self.FR_joints.append(sim.getObject( jointName[0 +i]) )
        self.MR_joints.append(sim.getObject( jointName[3 +i]) )
        self.HR_joints.append(sim.getObject( jointName[6 +i]) )
        self.FL_joints.append(sim.getObject( jointName[9 +i]) )
        self.ML_joints.append(sim.getObject( jointName[12+i]) )
        self.HL_joints.append(sim.getObject( jointName[15+i]) )


    self.FL_foot = sim.getObject('/FOOT_L0')
    self.ML_foot = sim.getObject('/FOOT_L1')
    self.HL_foot = sim.getObject('/FOOT_L2')
    self.FR_foot = sim.getObject('/FOOT_R0')
    self.MR_foot = sim.getObject('/FOOT_R1')
    self.HR_foot = sim.getObject('/FOOT_R2')

    self.FL_forcesensor = sim.getObject('/L0_fs')
    self.ML_forcesensor = sim.getObject('/L1_fs')
    self.HL_forcesensor = sim.getObject('/L2_fs')
    self.FR_forcesensor = sim.getObject('/R0_fs')
    self.MR_forcesensor = sim.getObject('/R1_fs')
    self.HR_forcesensor = sim.getObject('/R2_fs')




    # **************************************** #
    #                 Parameters               #
    # **************************************** #

    # == CPG
    # CPG 1
    self.o1_1 = 0.7856
    self.o2_1 = -0.5782
    self.a1_1 = 0
    self.a2_1 = 0
    self.MI_1 = 0.08
    self.w11_1 = 1.4
    self.w12_1 = 0.18 + self.MI_1
    self.w21_1 = -0.18 - self.MI_1
    self.w22_1 = 1.4

    # CPG 2
    self.o1_2 = -0.8376
    self.o2_2 = -0.6659
    self.a1_2 = 0
    self.a2_2 = 0
    self.MI_2 = 0.08
    self.w11_2 = 1.4
    self.w12_2 = 0.18 + self.MI_2
    self.w21_2 = -0.18 - self.MI_2
    self.w22_2 = 1.4

    # CPG 3
    self.o1_3 = -0.2865
    self.o2_3 =  0.8641
    self.a1_3 = 0
    self.a2_3 = 0
    self.MI_3 = 0.08
    self.w11_3 = 1.4
    self.w12_3 = 0.18 + self.MI_3
    self.w21_3 = -0.18 - self.MI_3
    self.w22_3 = 1.4

    # motor initial position (deg)
    self.FR_joints_init_position_deg = [30, 10, -60]
    self.MR_joints_init_position_deg = [0 ,  0, -60]
    self.HR_joints_init_position_deg = [-40, 10,-60]
    self.FL_joints_init_position_deg = [30, 10, -60]
    self.ML_joints_init_position_deg = [0 ,  0, -60]
    self.HL_joints_init_position_deg = [-40, 10,-60]

    # motor command
    self.FR_joints_target = [ 0,   0,     0]
    self.MR_joints_target = [ 0,   0,     0]
    self.HR_joints_target = [ 0,   0,     0]
    self.FL_joints_target = [ 0,   0,     0]
    self.ML_joints_target = [ 0,   0,     0]
    self.HL_joints_target = [ 0,   0,     0]

    # Force value
    self.FL_force_value = 0
    self.ML_force_value = 0
    self.HL_force_value = 0
    self.FR_force_value = 0
    self.MR_force_value = 0
    self.HR_force_value = 0

    # IMU value
    self.body_orientation = [0,0,0]

    # Simulation time
    self.simulation_time = 0
    
    pass
    
# ********************************************************* #
# ********************************************************* #
#                        ACTUATION                          #               
# ********************************************************* #
# ********************************************************* #

def sysCall_actuation():
    # **************************************** #
    #                   CPG                    #
    # **************************************** #

    # ==================== #
    #         CPG_1        #
    # ==================== #
    self.a1_1 = self.o1_1*self.w11_1 + self.o2_1*self.w12_1
    self.a2_1 = self.o2_1*self.w22_1 + self.o1_1*self.w21_1
    
    self.o1_1 = math.tanh(self.a1_1)
    self.o2_1 = math.tanh(self.a2_1)

    # ==================== #
    #         CPG_2        #
    # ==================== #
    self.a1_2 = self.o1_2*self.w11_2 + self.o2_2*self.w12_2
    self.a2_2 = self.o2_2*self.w22_2 + self.o1_2*self.w21_2
    
    self.o1_2 = math.tanh(self.a1_2)
    self.o2_2 = math.tanh(self.a2_2)

    # ==================== #
    #         CPG_3        #
    # ==================== #
    self.a1_3 = self.o1_3*self.w11_3 + self.o2_3*self.w12_3
    self.a2_3 = self.o2_3*self.w22_3 + self.o1_3*self.w21_3
    
    self.o1_3 = math.tanh(self.a1_3)
    self.o2_3 = math.tanh(self.a2_3)
    
    # ******************************************************************************************************************* #

    # **************************************** #
    #                Manual Gen                #
    # **************************************** #


    ## ====================================================================== ##
    ## ====================================================================== ##
    ## ====================================================================== ##
    ##                                                                        ##
    ##                              EDIT CODE HERE                            ##
    ##                                                                        ##
    ## ====================================================================== ##
    ## ====================================================================== ##
    ## ====================================================================== ##
    

    # ==== Sensor ==== #
    # force sensor
    FL_force = force_filter(self.FL_force_value)
    ML_force = force_filter(self.ML_force_value)
    HL_force = force_filter(self.HL_force_value)
    FR_force = force_filter(self.FR_force_value)
    MR_force = force_filter(self.MR_force_value)
    HR_force = force_filter(self.HR_force_value)
    # print(  FL_force,ML_force,HL_force,FR_force,MR_force,HR_force )

    # IMU 
    body_roll  = round(math.degrees(self.body_orientation[0]), 2)
    body_pitch = round(math.degrees(self.body_orientation[1]), 2)
    body_yaw   = round(math.degrees(self.body_orientation[2]), 2)
    # print(body_roll, body_pitch, body_yaw)

    ##########################################
    ##########################################
    ############ Your Code is Here ###########
    ##########################################
    ##########################################

    # CPG
    factor = 0.3
    cpgA_1 = self.o1_1 * factor
    cpgA_2 = self.o2_1 * factor

    cpgB_1 = self.o1_2 * factor
    cpgB_2 = self.o2_2 * factor

    cpgC_1 = self.o1_3 * factor
    cpgC_2 = self.o2_3 * factor


    if self.manual_gen:
        # self.LEG_joints_target = [M1, M2, M3]

        self.FL_joints_target = [      0,  cpgA_1,      0]
        self.ML_joints_target = [      0,  cpgA_1,      0]
        self.HL_joints_target = [      0,  cpgA_1,      0]
        self.FR_joints_target = [      0,  cpgA_1,      0]
        self.MR_joints_target = [      0,  cpgA_1,      0]
        self.HR_joints_target = [      0,  cpgA_1,      0]



    ## ====================================================================== ##
    ## ====================================================================== ##
    ## ====================================================================== ##
    ## ====================================================================== ##
    ## ====================================================================== ##
    
        # motor command
        for i in range(3):
            sim.setJointTargetPosition(self.FR_joints[i], self.FR_joints_target[i] + math.radians(self.FR_joints_init_position_deg[i]))
            sim.setJointTargetPosition(self.MR_joints[i], self.MR_joints_target[i] + math.radians(self.MR_joints_init_position_deg[i]))
            sim.setJointTargetPosition(self.HR_joints[i], self.HR_joints_target[i] + math.radians(self.HR_joints_init_position_deg[i]))
            sim.setJointTargetPosition(self.FL_joints[i], self.FL_joints_target[i] + math.radians(self.FL_joints_init_position_deg[i]))
            sim.setJointTargetPosition(self.ML_joints[i], self.ML_joints_target[i] + math.radians(self.ML_joints_init_position_deg[i]))
            sim.setJointTargetPosition(self.HL_joints[i], self.HL_joints_target[i] + math.radians(self.HL_joints_init_position_deg[i]))
        
    pass

# ********************************************************* #
# ********************************************************* #
#                          SENSING                          #            
# ********************************************************* #
# ********************************************************* #
def sysCall_sensing():
    self.simulation_time = sim.getSimulationTime()
    self.body_orientation = sim.getObjectOrientation(self.IMU, -1)

    # ********************************************** #
    #                   Force sensor                 #
    # ********************************************** #
    FL_force_value_list = sim.readForceSensor(self.FL_forcesensor)
    ML_force_value_list = sim.readForceSensor(self.ML_forcesensor)
    HL_force_value_list = sim.readForceSensor(self.HL_forcesensor)
    FR_force_value_list = sim.readForceSensor(self.FR_forcesensor)
    MR_force_value_list = sim.readForceSensor(self.MR_forcesensor)
    HR_force_value_list = sim.readForceSensor(self.HR_forcesensor)

    self.FL_force_value = FL_force_value_list[1][2]
    self.ML_force_value = ML_force_value_list[1][2]
    self.HL_force_value = HL_force_value_list[1][2]
    self.FR_force_value = FR_force_value_list[1][2]
    self.MR_force_value = MR_force_value_list[1][2]
    self.HR_force_value = HR_force_value_list[1][2]


    # ********************************************** #
    #                   Distance                     #
    # ********************************************** #
    distances = sim.getObjectPosition(self.IMU)
    # print(distances)
    
    if distances[2] > 0: # the robot is NOT fail --> update distance
        self.x_distance = round(distances[0] - 0.288, 2)
        self.y_distance = round(distances[1], 2)
        self.z_distance = round(distances[2], 2)
    
    # Check Time (30s) and Capture distance
    if(self.simulation_time < self.time_lap):
        print("Travel distance = "+ str(self.x_distance) + "m || Time = " + str(round(self.simulation_time, 1)) + "/"+ str(self.time_lap) +" s" )
        self.final_x_distance = self.x_distance
    else:
        print("Travel distance = "+ str(self.x_distance) + "m || Time = " + str(round(self.simulation_time, 1)) + "/"+ str(self.time_lap) + " s || " + "Final distance = " + str(self.final_x_distance) + "m")


    # ********************************************** #
    #                    Plot Graph                  #
    # ********************************************** #

    # ***********   CPG Graph  ************ #
    sim.setGraphStreamValue(self.cpg_graph, self.cpg_val1, self.o1_1)
    sim.setGraphStreamValue(self.cpg_graph, self.cpg_val2, self.o2_1)
    # sim.setGraphStreamValue(self.cpg_graph, self.cpg_val3, self.o1_2)
    # sim.setGraphStreamValue(self.cpg_graph, self.cpg_val4, self.o2_2)
    # sim.setGraphStreamValue(self.cpg_graph, self.cpg_val5, self.o1_3)
    # sim.setGraphStreamValue(self.cpg_graph, self.cpg_val6, self.o2_3)

    # *********** trajectories ************ #
    FL_foot_pos = sim.getObjectPosition(self.FL_foot, -1)
    ML_foot_pos = sim.getObjectPosition(self.ML_foot, -1)
    HL_foot_pos = sim.getObjectPosition(self.HL_foot, -1)
    FR_foot_pos = sim.getObjectPosition(self.FR_foot, -1)
    MR_foot_pos = sim.getObjectPosition(self.MR_foot, -1)
    HR_foot_pos = sim.getObjectPosition(self.HR_foot, -1)

    sim.setGraphStreamValue(self.traj_FL, self.traj_FL_x, FL_foot_pos[0])
    sim.setGraphStreamValue(self.traj_FL, self.traj_FL_y, FL_foot_pos[1])
    sim.setGraphStreamValue(self.traj_FL, self.traj_FL_z, FL_foot_pos[2])
    
    sim.setGraphStreamValue(self.traj_ML, self.traj_ML_x, ML_foot_pos[0])
    sim.setGraphStreamValue(self.traj_ML, self.traj_ML_y, ML_foot_pos[1])
    sim.setGraphStreamValue(self.traj_ML, self.traj_ML_z, ML_foot_pos[2])
    
    sim.setGraphStreamValue(self.traj_HL, self.traj_HL_x, HL_foot_pos[0])
    sim.setGraphStreamValue(self.traj_HL, self.traj_HL_y, HL_foot_pos[1])
    sim.setGraphStreamValue(self.traj_HL, self.traj_HL_z, HL_foot_pos[2])

    sim.setGraphStreamValue(self.traj_FR, self.traj_FR_x, FR_foot_pos[0])
    sim.setGraphStreamValue(self.traj_FR, self.traj_FR_y, FR_foot_pos[1])
    sim.setGraphStreamValue(self.traj_FR, self.traj_FR_z, FR_foot_pos[2])

    sim.setGraphStreamValue(self.traj_MR, self.traj_MR_x, MR_foot_pos[0])
    sim.setGraphStreamValue(self.traj_MR, self.traj_MR_y, MR_foot_pos[1])
    sim.setGraphStreamValue(self.traj_MR, self.traj_MR_z, MR_foot_pos[2])

    sim.setGraphStreamValue(self.traj_HR, self.traj_HR_x, HR_foot_pos[0])
    sim.setGraphStreamValue(self.traj_HR, self.traj_HR_y, HR_foot_pos[1])
    sim.setGraphStreamValue(self.traj_HR, self.traj_HR_z, HR_foot_pos[2])

    pass



# ********************************************************* #
# ********************************************************* #
#                          Clean Up                         #
# ********************************************************* #
# ********************************************************* #
def sysCall_cleanup():
    # do some clean-up here
    pass


# ********************************************************* #
# ********************************************************* #
#                    Additional Function                    #
# ********************************************************* #
# ********************************************************* #

def force_filter(force_val):
    force = np.tanh(relu(-force_val))
    return force
    
def relu(val):
    if val < 0:
        return 0
    else:
        return val
    
def sigmoid(val):
    return 1/(1 + np.exp(-val)) 