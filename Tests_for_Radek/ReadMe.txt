All files should work, as I have added in their dependent files too.

Here are some examples that should (hopefully) work, so that you can see the behaviour of the system:

tracking_with_angle_no_collapse(200,5001,10,10,10,100)
tracking_twelve_with_angle_inference(200,5001,10,10,10,100)

To change between the stationary/slow-moving case, just comment out/in the correct functions in lines 42-44. Also, Remember that in the N=12 case, if any of the drones begin by moving in the wrong direction, then the entire system will eventually cascade, so it's better to just restart the simulation.

The codes should hopefully be documented enough that you can change parameters if you wish. The only annoyance is that using 'animatedline' in Matlab means that there is a lot of slowdown if the graphics are expanded to full screen. However, I like them because you can use the zoom tool while they are updating.

I will send you a copy of the written equations tomorrow. There are definitely areas in the code that could be done more efficiently, but for now they work.


