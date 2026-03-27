 try:
#            fg.sw_trigger() 
#            fg.start([buffer_id.AMPLITUDE_IMAGE, buffer_id.RADIAL_DISTANCE_IMAGE, buffer_id.CONFIDENCE_IMAGE, buffer_id.XYZ])
            [ok, frame]=fg.wait_for_frame().wait_for(150)
            print("trying to get frame, success:", ok)
        except:
            print('could not connect to camera')
            cam_was_down=1
            continue
        else:
            print("fg status:", fg.is_running())
 #           continue
 #       fg.on_error(continue)
#        [ok, frame]=fg.wait_for_frame().wait_for(5)
        if ok:
            #print("OK!!", frame.timestamps()
            tmestamps=frame.timestamps()
            #continue
#        while not  :
#            continue
        if not ok:
            continue
            #raise RuntimeError("Timeout while waiting for a frame.")
        #frame1 = getter(buf)
        #NORM_AMPLITUDE_IMAGE, buffer_id.RADIAL_DISTANCE_IMAGE, buffer_id.XYZ]  )
         
      ####live script
        
        a=np.uint8(frame.get_buffer(buffer_id.AMPLITUDE_IMAGE))
       # frame1=frame.get_buffer(buffer_id.NORM_AMPLITUDE_IMAGE)
        frame1=np.uint8(frame.get_buffer(buffer_id.RADIAL_DISTANCE_IMAGE))
        #a=frame.get_buffer(buffer_id.NORM_AMPLITUDE_IMAGE)
        rows = a.shape[1]
        cols = a.shape[0]
#        print(rows)
#        print(cols)
#        print(a.max()) 
        
        for x in range(0, cols - 1):
            for y in range(0, rows -1):
                #print(a[x,y])
                pixel_value=a[x,y]
                if pixel_value>=16200:
                    a[x,y]=0
                if x>=215:
                    a[x,y]=0
#        print(a.max())     
        # Define the desired range
        array=a
        a, b = 0, 255

        # Min and max of the array
        array_min = np.min(array)
        array_max = np.max(array)

        # Normalize to the range [a, b]
        frame2 = a + ( (array - array_min) * (b - a) / (array_max - array_min) )
        #frame2=cv2.normalize(a, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
        
        frame3=np.uint8(frame.get_buffer(buffer_id.CONFIDENCE_IMAGE))

        #print(frame2.shape)
        pt=frame.get_buffer(buffer_id.XYZ)
#        print("pt", np.max(pt))
        '''
        ###from file script
#        pth=
#        print(pth)
        root="/home/niall/Pictures/listons/2023_12_05/AM/"
        while (len(list(pathlib.Path(root+str(s)).glob("*_i"+str(s)+"_s"+str(idx)+"_amp.png")))==0) and s<1000:
            s=s+1
            #break
        
        pth=list(pathlib.Path(root+str(s)).glob("*_i"+str(s)+"_s"+str(idx)+"_amp.png"))[0]
            #idx=1i doesnt reset need to fox i recording program
        frame1=np.uint8(Image.open(pth))
        frame2=np.uint8(Image.open(str(pth).replace("amp", "dist")))
        frame3=np.uint8(Image.open(str(pth).replace("amp", "conf")))
        pt=np.load(str(pth).replace("amp.png", "pcd2.npy"))
        np.set_printoptions(threshold=sys.maxsize)
#        print(pt.shape)
#        print(pt)
        pt = pt.reshape((176,132,3))
        
        idx=idx+1
#        print(pt.shape)
#        print(pt)
        '''
        
        
        
         ###savevscript
        
        if(int(shoot) != prev_shoot  ):
            prev_shoot=int(shoot)
            if(int(shoot) == 1 ):
                index=index+1
                subindex=0
                print ("Index is", index)
                today = datetime.date.today()  
                datestamp=today.strftime("%Y_%m_%d")
                timestamp=today.strftime("%Y_%m_%d_%H_%M_%S")
                #todaystr = today.isoformat()   
                #os.makedirs(
                os.makedirs('/mnt/tagmii1/'+datestamp+'/'+ampm+'/'+str(index), exist_ok=True)
            
       
        #filestr="/media/niall/tagmii1/" + str(idx)
        filestr='/mnt/tagmii1/'+datestamp+'/'+ampm+'/'+str(index)+'/'+timestamp+'_i'+str(index)+'_s'+str(subindex)
  
  #uncooment here to save images   
         f1=frame1
         f2=frame2
         f3=frame3
         np.save(filestr+'_f1', f1)
         np.save(filestr+'_f2', f2)
         np.save(filestr+'_f3', f3)
         
         ptw = pt.reshape(pt.shape[:-3] + (-1, 3))
         np.save(filestr+'_pcd2', ptw)
#        im1 = Image.fromarray(frame1)
#            
#        im1.save(filestr+'_amp.png')
#        im2 = Image.fromarray(frame2)
#        im2.save(filestr+'_dist.png')
#        im3 = Image.fromarray(frame3)
#        im3.save(filestr+'_conf.png')
#        ptw = pt.reshape(pt.shape[:-3] + (-1, 3))
#        np.save(filestr+'_pcd2', ptw)
        idx=idx+1
#        frame1=cv2.normalize(frame.get_buffer(buffer_id.NORM_AMPLITUDE_IMAGE), None, 0,256, cv2.NORM_MINMAX, cv2.CV_8U)
#       # frame1=frame.get_buffer(buffer_id.NORM_AMPLITUDE_IMAGE)
#        frame2=cv2.normalize(frame.get_buffer(buffer_id.RADIAL_DISTANCE_IMAGE), None, 0,256, cv2.NORM_MINMAX, cv2.CV_8U)
#       frame3=cv2.normalize(frame.get_buffer(buffer_id.CONFIDENCE_IMAGE), None, 0,256, cv2.NORM_MINMAX, cv2.CV_8U)
#        pt=frame.get_buffer(buffer_id.XYZ)
        # Adjusts the brightness by adding 10 to each pixel value 
        brightness = -0 
        # Adjusts the contrast by scaling the pixel values by 2.3 
        contrast = 0.6
       # frame1 = cv2.addWeighted(frame1, contrast, np.zeros(frame1.shape, frame1.dtype), 0, brightness) 
        #frame1 = cv2.addWeighted(frame2, 0.6, np.zeros(frame2.shape, frame2.dtype), 0, 0) 
        frame1=frame2*0.8
        #frame3 = cv2.addWeighted(frame3, -1.0, np.zeros(frame3.shape, frame3.dtype), 0, 60)   
        
        
        c = frame3# np.array([1, 4, 2, 5, 7, 4, 2, 5, 6, 7, 7, 2, 5])
        mapping = np.arange(c.max() + 1)
        map_from = np.array([45, 48, 56,  61])
        map_to = np.array([15, 60, 180, 180])
        mapping[map_from] = map_to
        frame3=np.uint8(mapping[c])
        #pressed_key = cv2.waitKey(1)
        #cv2.imshow("teat_detector", frame1)
        image_np = live_image_into_numpy_array(frame1, frame3, frame2)#bgr
         # Things to try:231
    # Flip horizontally
        # image_np = np.fliplr(image_np).copy()

        # Convert image to grayscale
        # image_np = np.tile(
        #     np.mean(image_np, 2, keepdims=True), (1, 1, 3)).astype(np.uint8)
        
        # The input needs to be a tensor, convert it using `tf.convert_to_tensor`.
        input_tensor = tf.convert_to_tensor(image_np)
        # The model expects a batch of images, so add an axis with `tf.newaxis`.
        input_tensor = input_tensor[tf.newaxis, ...]

        detections = detect_fn(input_tensor)

        # All outputs are batches tensors.
        # Convert to numpy arrays, and take index [0] to remove the batch dimension.
        # We're only interested in the first num_detections.
        num_detections = int(detections.pop('num_detections'))
        detections = {key: value[0, :num_detections].numpy()
                      for key, value in detections.items()}
        detections['num_detections'] = num_detections
        

        # detection_classes should be ints.
        detections['detection_classes'] = detections['detection_classes'].astype(np.int64)
       # print(int(frame1.shape[1]))
        cam_pos = list()
        defects=list()
        boxes=detections['detection_boxes'] 
        if(len(boxes)!=0):
            #print (boxes)
            indices = np.argsort(boxes[:,0])
            #print(indices)
    #Then you can use these indices to sort the boxes, scores, and classed, as follows:

    #sorted_scores = scores[0][indices]
            sorted_boxes = boxes#[0][indices]
            #detections['detection_classes'] = detections['detection_classes'][0] [indices]
    #If you wanted to, for example, sort by xmax instead, you'd use np.argsort(boxes[0][:,2]). You can play around with using 0-3 to sort by xmin, ymin, xmax, and ymax.
            #print(int(frame1.shape[1]))
            width = int(frame1.shape[1])
            height= int(frame1.shape[0])
            #print(pt)
            if(len(boxes)!=0):
                #print (detections)
                indices = np.argsort(boxes[:,0])
                #print(indices)
        #Then you can use these indices to sort the boxes, scores, and classed, as follows:

        #sorted_scores = scores[0][indices]
                sorted_boxes = boxes#[0][indices]
                #detections['detection_classes'] = detections['detection_classes'][0] [indices]
        #If you wanted to, for example, sort by xmax instead, you'd use np.argsort(boxes[0][:,2]). You can play around with using 0-3 to sort by xmin, ymin, xmax, and ymax.
                #print(int(frame1.shape[1]))
                width = int(frame1.shape[1])
                height= int(frame1.shape[0])
                #print(pt)
                for i in range(len(boxes)):
                   if detections['detection_classes'][i]<=2:
                       
                       ymin = int((boxes[i,0]*height))
                       xmin = int((boxes[i,1]*width))
                       ymax = int((boxes[i,2]*height))
                       xmax = int((boxes[i,3]*width))
    #                   xmin = (int(boxes[i,0]*width))+4
    #                   ymin = (int(boxes[i,1]*height))
    #                   xmax = (int(boxes[i,2]*width))
    #                   ymax = (int(boxes[i,3]*height))
                      ### print(xmin, xmax, ymin, ymax)
                       a=int(xmin+(xmax-xmin)/2)
                       b=int(ymin+((ymax-ymin)/2))
                       
                       
                       x=pt[b, a][0]
                       y=pt[b, a][1]
                       z=pt[b, a][2]
                     #j  print(x,y,z)
                       c=0
                       
    #                   for alpha in range(xmin, xmax):
    #                    for beta in range(ymin, ymax):
                       #3d    
                       while(x==0 and c<5):
                        b=b-1
                        c=c+1
                        x=pt[b, a][0]
                        y=pt[b, a][1]
                        z=pt[b, a][2]
                       if a>70 and a <105:# and b >25 and b<70 and ymax-ymin<22 and xmax-xmin <18 and z<950 and z>550:
                           #print("sending", boxes, detections['detection_classes'][i], a, b, x, y, z)
                           cam_dict={'XYZ'+str(i+1): {'X': str(x), 'Y': str(y),'Z': str(z)}}
                                                     #cam_pos.append(cam_dict)
                           cam_pos.append(cam_dict)
                           defects.append([a, b])
                           #break
                       else:
                             detections['detection_scores'][i]=0
    #                        
                        
                   
#                   print("sending", boxes, detections['detection_classes'][i], a, b, x, y, z)
#                   cam_dict={'XYZ'+str(i+1): {'X': str(x), 'Y': str(y),'Z': str(z)}}
#                                             #cam_pos.append(cam_dict)
#                   cam_pos.append(cam_dict)
                   #break
        
#                   print("sending", boxes, detections['detection_classes'][i], a, b, x, y, z)
#                   cam_dict={'XYZ'+str(i+1): {'X': str(x), 'Y': str(y),'Z': str(z)}}
#                                             #cam_pos.append(cam_dict)
#                   cam_pos.append(cam_dict)
#        # bit for sending telegram of obj results
#        if(len(cam_pos) != 0):
#            print(cam_pos)
#            telegram = create_xml(cam_pos)
#                
#            UDPClientSocket.sendto(telegram, ('127.0.0.1', 59152))
#            print("sent", telegram)
        #print(defects)
        process_teats(defects[:4], pt)
        
def process_teats(far_points, pt):
            
            #print("Centroid : " + str(cnt_centroid) + ", farthest Point : " + str(far_points))
            
            cam_pos = list()
            teat_cntr=0
            teats = np.zeros( shape=( 4 , 3 ) )

            ###print("far_points", far_points)
            if False:#far_points != None:# and (far_points <= 100).any()and (far_points >= 20).any():
                far_points=sorted(far_points, key=lambda x: x[0], reverse=True)
                #print("farthest Point sorted: " + str(far_points))
                far_points_history=far_points
                ###print("far_points_history", far_points_history)
                for i in range(len(far_points)):
                    
                    far_point=tuple(far_points[i])
                    
                    #print(far_points[i])
                   # cv2.circle(frame1, far_point, 5, [255, 128, 128], -1)
                    #print("traverse_point : " + str(cnt_centroid) + ", farthest Point : " + str(far_point))
                    if np.any(far_points[i]):
                        
                        ###print("iiiii "+ str(i)+ str(far_point))
                        for j in range(len(traverse_points)):
                        
                        #for traverse_point in traverse_points:
                            traverse_point=tuple(traverse_points[j])
                           ### print("jjj "+ str(j)+ str(traverse_point)+ "age " + str(traverse_points_age[j]))
                            #traverse_point_age=tuple(traverse_points_age[j])
                            for k in range(len(traverse_points)):
                                if j == k:
                                    break
                                ###print("kkk "+ str(j)+ str(traverse_point)+ "age " + str(traverse_points_age[j]))
                                if np.allclose(traverse_points_age[k], traverse_points_age[j], rtol=0.1, atol=0.1, equal_nan=False) or traverse_points_age[k]>max_age:
                                    traverse_points[k]=(0, 0)
                                    traverse_points_age[k]=0
                                    print("delete old tp")
                            
        #                    if len(traverse_points) < 4:
        #                        np.append(traverse_points, far_point)
                            if np.allclose(far_point, traverse_point, rtol=0.1, atol=0.1, equal_nan=False):
        #                        far_points[i], far_points[j] = far_points[j], far_points[i] 
                                traverse_points[j]=far_point
                                traverse_points_age[j]=0
                                print("close")
                                #np.delete(traverse_points, traverse_point)
                                #np.append(traverse_points, far_point)
                                break
                            elif not np.any(traverse_points[j]) or traverse_points_age[j]>max_age:
                                traverse_points[j]=far_point
                                traverse_points_age[j]=0
                                print("fill empty tp")
                                #np.delete(traverse_points, traverse_point)
                                #np.append(traverse_points, far_point)
                                break
                            
                                #np.delete(traverse_points, traverse_point)
                                #np.append(traverse_points, far_point)
                                #break
                            
                                
                            #                        elif j == len(traverse_points):
                            #                            traverse_points.pop(0)
                            #                            np.append(traverse_points, far_point)
                                traverse_points_age[j]+=1
                                h=16
                                w=16
                                y1 = traverse_points[j][1] + 6
                                y2 = traverse_points[j][1] - (h)
                                x1 = traverse_points[j][0] - int(w/2)
                                x2 = traverse_points[j][0] + int(w/2)
                                print(x1)
                                # Draw the bounding box on the frame
#                                cv2.rectangle(
#                                    frame1, (x1,y1),
#                                    (x2,y2), (0,255,0),
#                                    2
#                                )
#                                gray = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)
#                                to_canny = cv2.GaussianBlur(gray, (5, 5), 3)
#                                
#                                edges = cv2.Canny(to_canny, 0, 127)
#                                print (to_canny)
#                                print (edges)
#                                # Since the edges are only a 2-channel frame,
#                                # we can overlay it on to each channel in the
#                                # original frame
#                                frame1[y1:y2, x1:x2, 0] = edges[y1:y2, x1:x2]
#                                frame1[y1:y2, x1:x2, 1] = edges[y1:y2, x1:x2]
#                                frame1[y1:y2, x1:x2, 2] = edges[y1:y2, x1:x2]



                    #draw_circles(frame1, traverse_points)
                    
                    
                    a=int(far_point[1])
#                    if (far_point[0]<cnt_centroid[0]):
#                        b=int(far_point[0])
#                    else:
                    b=int(far_point[0])
                    x=pt[a, b][0]
                    y=pt[a, b][1]
                    z=pt[a, b][2]
                    print("index: "+str(i))
                    print("x "+str(x))
                    print("y "+str(y))
                    print("z "+str(z))


                    if x!=0 and y!=0 and z!=0:
                        teats[teat_cntr] = np.array([x,y,z])
                        teat_cntr=teat_cntr+1
                    #wd_dict={'Digout':{'o1': str(wd_bit)}}
           #######         cam_dict={'XYZ'+str(i+1): {'X': str(x), 'Y': str(y),'Z': str(z)}}
                    #cam_pos.append(cam_dict)
                  #####  cam_pos.append(cam_dict)
            if (teat_cntr>=2):
                udder_midpt= np.sum(teats, axis=0) / teat_cntr
                print("udder_midpt", udder_midpt)
                print("teat_cntr", teat_cntr, teats.shape[0], teats)
#                x=udder_midpt[0]
#                if(teat_cntr==1)
#                    x=
                y=udder_midpt[1]
                z=udder_midpt[2]
                cam_dict={'XYZ1': {'X': str(x), 'Y': str(y),'Z': str(z)}}
                #cam_dict={'XYZ1': {'X': str(x), 'Y': str(y),'Z': str(z)},'XYZa': {'X': str(teats[0][000000000000000000000]), 'Y': str(teats[0][1]),'Z': str(teats[0][2])}},'XYZb': {'X': str(teats[1][0]), 'Y': str(teats[1][1]),'Z': str(teats[1][2])}, 'XYZc': {'X': str(teats[2][0]), 'Y': str(teats[2][1]),'Z': str(teats[2][2])}, 'XYZd': {'X': str(teats[3][0]), 'Y': str(teats[3][1]),'Z': str(teats[3][2])}}
                cam_pos.append(cam_dict)
                

            elif (teat_cntr==1):
                x=teats[0][0]
    #                if(teat_cntr==1)
    #                    x=
                y=teats[0][1]
                z=teats[0][2]
                cam_dict={'XYZ1': {'X': str(x), 'Y': str(y),'Z': str(z)}}
                cam_pos.append(cam_dict)
#            else
#                cam_dict={'XYZ1': {'X': str(1), 'Y': str(2),'Z': str(3)}}
#                cam_pos.append(cam_dict)
#            x=50
 #           y=10
  #          z=500
   #         cam_dict={'XYZ1': {'X': str(x), 'Y': str(y),'Z': str(z)}}
    #        cam_pos.append(cam_dict)
           
            if(len(cam_pos) != 0):
                cam_dict={'XYZa': {'X': str(teats[0][0]), 'Y': str(teats[0][1]), 'Z': str(teats[0][2])}}
                cam_pos.append(cam_dict)
                cam_dict={'XYZb': {'X': str(teats[1][0]), 'Y': str(teats[1][1]), 'Z': str(teats[1][2])}}
                cam_pos.append(cam_dict)
                cam_dict={'XYZc': {'X': str(teats[2][0]), 'Y': str(teats[2][1]), 'Z': str(teats[2][2])}}
                cam_pos.append(cam_dict)
                cam_dict={'XYZd': {'X': str(teats[3][0]), 'Y': str(teats[3][1]), 'Z': str(teats[3][2])}}

                cam_pos.append(cam_dict)
                print(cam_pos)
                telegram = create_xml(cam_pos)
                    
                UDPClientSocket.sendto(telegram, ('127.0.0.1', 59152))
                print("sent", telegram)     
