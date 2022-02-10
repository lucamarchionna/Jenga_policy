#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32, String    #Message is single float number
import numpy as np
import random
from tracker_visp.msg import *
from tracker_visp.srv import *

global block_choice, init_layer_orientation
global block_tower, layer_matrix

init_layer_orientation = ''


def validate_tower(new_tower):
    columns = new_tower.shape[0]

    valid_tower = np.delete(new_tower, [0, 1, 2], 0) #=0 stay for delete columns

    return valid_tower

def InitOrientationCallback(oriention):
    global init_layer_orientation

    rospy.loginfo("Entered")
    
    if oriention.data == 'sx' or oriention.data == 'dx':
        init_layer_orientation = oriention.data

    else:
        rospy.logwarn("Please provide a correct orientation convention")

def compute_orientation(layer):
    initial_first_layer = 18

    possible_orient = ['dx', 'sx']

    layer = int(layer)

    if layer % 2 == 0:
        layer_orient = init_layer_orientation

    else:
        if init_layer_orientation == possible_orient[0]:
            layer_orient = possible_orient[1]

        else:
            layer_orient = possible_orient[0]

    rospy.loginfo("Completed the policy with the information of orientation")

    return layer_orient

def insert_priority(tower, first, second):

    columns = tower.shape[0]
    layer = np.zeros((columns, 2), dtype=int) #priority, analyzed

    for level in first:
        layer[columns-(level)][0] = 1

    for level in second:
        layer[columns-(level)][0] = 2
        
    return layer

def get_random_layer():
    global layer_matrix

    columns = layer_matrix.shape[0]

    valid_layer = []
    
    for level in range(columns):
        if layer_matrix[level][0] == 1 and layer_matrix[level][1] == 0 :
            valid_layer.append(columns-level)

    if not valid_layer:
        for level in range(columns):
            if layer_matrix[level][0] == 2 and layer_matrix[level][1] == 0 :
                valid_layer.append(columns-level)
    
    random_num = random.choice(valid_layer)
    
    return random_num

def get_random_block(layer):
    global block_tower

    columns = block_tower.shape[0]

    complete_blocks = ['sx', 'cx', 'dx']
    possible_blocks = []

    rectified_layer = columns-layer   
 
    for i in range(3):
        if block_tower[rectified_layer][i] == 0:
            possible_blocks.append(complete_blocks[i])

        else:
            pass
    
    random_block = random.choice(possible_blocks)
    print("I've chosen the block",random_block, "in the layer n.", layer)

    return random_block

def totowerpos_int(block_loc):
    pos = 0
    if block_loc.position == 'dx':
        pos = 1

    elif block_loc.position == 'cx':
        pos = 0

    elif block_loc.position == 'sx':
        pos = -1

    else:
        rospy.logwarn("Please provide a correct position convention")

    return pos

def update_block(choice):
    global block_tower

    columns = block_tower.shape[0]

    position = totowerpos_int(choice)
    block_tower[columns-choice.layer][position+1] = 1

def change_block_same_layer():
    global block_tower, block_choice

    columns = block_tower.shape[0]
    layer = block_choice.layer

    complete_blocks = ['sx', 'cx', 'dx']
    possible_blocks = []

    rectified_layer = columns-layer   
 
    for i in range(3):
        if block_tower[rectified_layer][i] == 0:
            possible_blocks.append(complete_blocks[i])

        else:
            pass

    if possible_blocks:
        random_block = random.choice(possible_blocks)
    
    else:
        rospy.loginfo("No more blocks on the same layer")
        random_block = ''

    return random_block

def update_layer(single_block):
    global layer_matrix

    columns = layer_matrix.shape[0]

    layer_matrix[columns-single_block.layer][1] = 1

def take_another_block_and_update(block_to_update):
    update_block(block_to_update.location)
    update_layer(block_to_update.location)

    chosen_layer = get_random_layer()
    chosen_block = get_random_block(chosen_layer)
    block_choice = toLocMsg(chosen_layer, chosen_block) 

    return block_choice

def take_another_block(block_to_update):

    chosen_layer = get_random_layer()
    chosen_block = get_random_block(chosen_layer)
    block_choice = toLocMsg(chosen_layer, chosen_block) 

    return block_choice

def toLocMsg(layer, block_position):
    block_msg = location()
    block_msg.layer = layer
    block_msg.position = block_position

    orient = compute_orientation(layer)
    block_msg.orientation = orient

    return block_msg

def BlockEstimationCallback():
    global block_choice
    
    rospy.wait_for_service('/BlockEstimation')
    req = rospy.ServiceProxy('/BlockEstimation', BlockEstimation)

    resp = req(block_choice)

    return resp.cTo_est
 
def PoseEstimationCallback(img, cam, cTo_est):
    rospy.wait_for_service('/PoseEstimation')
    req = rospy.ServiceProxy('/PoseEstimation', PoseEstimation)

    # cao_path, init_pose = req(img, cam, cTo_est)
    resp = req(img, cam, cTo_est)

    return resp.caoFilePath, resp.initPose

def create_block_status(tower):
    global block_tower

    columns = tower.shape[0]
    block_matrix = np.zeros((columns, 3), dtype=int) #0 for untested, 1 for tested

    block_tower = np.zeros((columns, 3), dtype=int)
    
    return block_matrix

def forcebasedCallback(service):
    global block_tower, block_choice

    if service.BeyondFmax:
        update_block(block_choice)
        block_choice.position = change_block_same_layer()
        
        if not block_choice.position:
            take_another_block_and_update()


    else:
        take_another_block_and_update()

    print(block_choice)
    return ForceBasedDecisionResponse(
        block_choice
    )

def yolactCallback(service):
    global block_choice

    success = False
    count = 0

    max_numb = 10

    while not success and count < max_numb:
        img = service.image
        cam = service.camInfo

        #second service - BlockEstimation
        cTo_est = BlockEstimationCallback()

        #third service - PoseEstimation
        cao_path, init_pose = PoseEstimationCallback(img, cam, cTo_est)

        if not cao_path.data:
            success = False

        else:
            success = True
        
        count = count +1

        if count == max_numb and not success:
            block_choice = take_another_block(cTo_est)
            count = 0

        else:
            pass 

    return YolactInitializeCaoPoseResponse(
        cao_path, init_pose
    )

    
if __name__ == '__main__':
    policy = rospy.init_node("Jenga_policy", anonymous=True)

    tower_init = np.zeros((18, 3), dtype=int)
    
    tower = validate_tower(tower_init)

    subOrientation = rospy.Subscriber('/init_orientation', String, InitOrientationCallback)

    first_priority = [4, 5, 9, 10, 11, 12, 13] 
    second_priority = [1, 2, 3, 6, 7, 8, 14, 15]
    layer_matrix = insert_priority(tower, first_priority, second_priority)  
    block_matrix = create_block_status(tower)

    chosen_layer = get_random_layer()
    chosen_block = get_random_block(chosen_layer)

    #Wait until it is available 
    arrived = False

    while not arrived and not rospy.is_shutdown():
        if init_layer_orientation == 'dx' or init_layer_orientation == 'sx':
            arrived = True
            block_choice = toLocMsg(chosen_layer, chosen_block)   
            rospy.loginfo("Continuing...")
        else:
            pass
    

    rospy.sleep(5)
  
    #first service - YolactInitializeCaoPose
    serv_yolact = rospy.Service('YolactInitializeCaoPose', YolactInitializeCaoPose, yolactCallback)
    serv_force = rospy.Service('/ForceBasedDecision', ForceBasedDecision, forcebasedCallback)

    #publish the selected layer for the detection
    print(block_choice)

    #add constrained blocks
    
    rospy.spin()

        