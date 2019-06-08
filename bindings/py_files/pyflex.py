# Written by Henry M. Clever. November 15, 2018.


import bindings as flexbind
from time import time
from random import random
import numpy as np

main_loop_quit = False

flexbind.chooseScene(0)

flexbind.initialize()

step = 0
# print(flexbind.getNumParticles())
# print(flexbind.getNumInstances())
# print(flexbind.getAllCenters())
while main_loop_quit == False:
    # act = np.random.uniform([-2, -2, -1, -1], [2, 2, 1, 1], 4)
    # act[2::] / np.linalg.norm(act[2::])
    act = np.random.rand(100)
    s1 = flexbind.update_frame(act)
    orig = time()
    # if(step%100==0):
    #     flexbind.setSceneRandSeed(0)
    #     flexbind.resetScene()
    step += 1
    # pos_array = np.zeros((1000, 3))
    #
    # for i in range(0, 1000):
    #     pos_array[i, 0] = flexbind.grab_x_pos_particle(i)
    #     pos_array[i, 0] = flexbind.grab_y_pos_particle(i)
    #     pos_array[i, 2] = flexbind.grab_z_pos_particle(i)
    # print(flexbind.getState())

    main_loop_quit = flexbind.sdl_main()

flexbind.destroy_scene()

#
#
# print main_loop_quit
#
# print "got here in python!!!"
