from lib_robot_transformations import *
import tensorflow as tf
import numpy as np

def build_kuka_fr(fr): # builds KUKA_frame from (x, y, z, a, b, c)
    kuka_fr = KUKA_frame(x= fr[0], y = fr[1], z = fr[2], a = fr[3], b = fr[4], c = fr[5])
    return kuka_fr
############
def build_kuka_fr_for_cam(fr): # builds KUKA_frame from (x, y, z, a, b, c)
    kuka_fr = KUKA_frame(x= fr[0], y = fr[1], z = fr[2], a = fr[5], b = fr[4], c = fr[3])
    return kuka_fr
##############

def build_rot_mtx_vec_A(kuka_fr_1, kuka_fr_2): 
    res_rot_mtx = kuka_fr_1.rot_mtx().inv() * kuka_fr_2.rot_mtx()
    return res_rot_mtx.mtx

def build_rot_mtx_vec_B(cam_fr_1, cam_fr_2): 
    res_rot_mtx = cam_fr_1.rot_mtx() * cam_fr_2.rot_mtx().inv()
    return res_rot_mtx.mtx

def make_unique_indexes(num_of_positions):
    indexes_arr = list()
    for i in range(num_of_positions):
        if i < num_of_positions:
            for j in range(i + 1, num_of_positions, 1):  
                indexes_arr.append((i, j))

    return indexes_arr       

def make_non_unique_indexes(num_of_positions):
    indexes_arr = list()
    for i in range(num_of_positions):        
        for j in range(num_of_positions): 
            if i != j:
                indexes_arr.append((i, j))

    return indexes_arr   

# BOULD NP ARRAY OF ROT MATRICES
def build_rot_mtx_arr_from_source(source_frames_list):
    data = np.empty(shape = (len(source_frames_list), 4, 4)).astype(np.float32)
    for i_num, i_fr in enumerate(source_frames_list):
        data[i_num] = build_kuka_fr(i_fr).rot_mtx().mtx
        
    return data

# CREATE TF_DATASET BASED ON NP ARRAY OF ROT MATRICES
def create_dataset_shuffled(np_arr, buffer_size, batch_size ):
    train_dataset = tf.data.Dataset.from_tensor_slices(np_arr)
    train_dataset = train_dataset.shuffle(buffer_size).batch(batch_size) # DON'T use SHUFFLE - need use the same from A and B
    # train_dataset = train_dataset.batch(batch_size)
    return train_dataset

def add_row(tf_var): 
    """
    Add row of [0, 0, 0, 1] at the bottom of x_mtx varible.
    Make it's shape [4,4] from [3,4]
    This is necessary to make multiplication of rotation matrices
    """
    a = tf.unstack(tf_var)
    row = tf.constant( np.array([0, 0, 0, 1]).astype(np.float32))
    a.append(row)
    tf_var = tf.stack(a)
    return tf_var

# def x_mtx_2_kuka_frame(x_mtx ):
#     """
#     From [4,4] shaped Rotation matrix -> getting KUKA_frame (X, Y, Z, A, B, C))
#     """
#     temp = np.zeros(shape=(4,4))
#     temp[:3, :] = x_mtx.numpy()
#     temp[3, :] = [0, 0, 0, 1]
#     x_mtx_obj = Rotation_matrix(mtx = temp, parent_type = 'KUKA')
#     x_fr = x_mtx_obj.extract_frame()
#     return np.array([x_fr.x, x_fr.y, x_fr.z, x_fr.a, x_fr.b, x_fr.c])

def x_mtx_2_kuka_frame(x_mtx ):
    """
    From [4,4] shaped Rotation matrix -> getting KUKA_frame (X, Y, Z, A, B, C))
    """
    temp = np.zeros(shape=(4,4))
    temp[:3, :] = x_mtx.numpy()
    temp[3, :] = [0, 0, 0, 1]
    x_mtx_obj = Rotation_matrix(mtx = temp, parent_type = 'KUKA')
    x_fr = x_mtx_obj.extract_frame()
    return np.array([x_fr.x * 1000, x_fr.y * 1000, x_fr.z * 1000, x_fr.a, x_fr.b, x_fr.c])

def plot_frame_history(x_fr_history ):
    """
    Visualization of evolving X, Y, Z, A, B, C during the all process of optimization
    """
    fig, axs = plt.subplots(3, 2, figsize=(120, 60))
    # for i, alpha in enumerate(np.linspace(0, 1, n)):
    #     curr_vec = v_1 * (1-alpha) + v_2 * alpha
    #     image = generate_data(curr_vec, generator)[0]
        # print(type(image))
    axs[0][0].set_title('X axis')
    axs[0][0].plot(x_fr_history[:, 0])
    
    axs[1][0].set_title('Y axis')
    axs[1][0].plot(x_fr_history[:, 1])
        
    axs[2][0].set_title('Z axis')
    axs[2][0].plot(x_fr_history[:, 2])

        
    axs[0][1].set_title('A angle')
    axs[0][1].plot(x_fr_history[:, 3])

    axs[1][1].set_title('B angle')
    axs[1][1].plot(x_fr_history[:, 4])

    axs[2][1].set_title('BCangle')
    axs[2][1].plot(x_fr_history[:, 5])
    # axs[0].axis('off')

def loss(A_batch, B_batch, x_mtx):  
    
    tmp_x_mtx = add_row(x_mtx)
    # TRANSL LOSS
    ax = tf.linalg.matmul(A_batch, tmp_x_mtx, transpose_b=False) 
    axb = tf.linalg.matmul(ax, B_batch,  transpose_b=False)
    transl_axb = axb[:, :-1, -1] # shape[batch, 3]

    # calculation of euclidean distance
    b = tf.reshape(transl_axb, [-1,2, 3])
    substracted = tf.math.subtract(b[:, 0, :], b[:, 1, :])
    squared = tf.square(substracted)
    summed = tf.math.reduce_sum(squared, axis = 1)
    sqrtted = tf.math.sqrt(summed)
    loss_transl = tf.math.reduce_mean(sqrtted)

    return loss_transl

def train_step(x_mtx, loss_func, A_batch, B_batch):
    """
    Optimization step implemented without .fit()

    """
    #################################################
    with tf.GradientTape() as tape: # recording of the gradients from every step of optimization 
        loss = loss_func(A_batch, B_batch, x_mtx)
    
    grad = tape.gradient(loss, x_mtx) # dLoss/dImage
    opt.apply_gradients([(grad, x_mtx)]) # one step of gradient descent: image = image - lambda*dLoss/dImage                                         
    ###################################################
    return loss.numpy()
    
def train(x_mtx, loss_func, dataset, num_epochs = 1000):
    # init the iterators of DATASETs
    loss_history = list()
    x_fr_history = np.array([0,0,0,0,0,0])
    # datasetA_iterator = iter(dataset_A)
    # datasetB_iterator = iter(dataset_B)

    for epoch in range(num_epochs):
        # create batches of data
        for batch in dataset:   
            A_batch = batch[:, :, :4]
            B_batch = batch[:, :, 4:]
            loss_value = train_step(x_mtx, loss_func, A_batch, B_batch)
            
            
        loss_history.append(loss_value)
        x_fr_history = np.vstack( (x_fr_history, x_mtx_2_kuka_frame(copy.deepcopy(x_mtx)))) # add postprocessed x_mtx to history

        if epoch % 2 == 0:
            display.clear_output(wait=True)
            print('Epoch {}: loss: {}'.format(epoch, loss_value))
            print(x_mtx.numpy())
            plt.plot(loss_history)
            plt.show()

        if loss_value < 0.0001:
            break

    print(x_mtx_2_kuka_frame(x_mtx))
    return x_mtx_2_kuka_frame(x_mtx), loss_history, x_fr_history
    
def optimize(cam_frames_list, rob_frames_list):
    # 1. build np arrays full of frames converted to transformation matrices
    data_robot = build_rot_mtx_arr_from_source(rob_frames_list)
    data_camera = build_rot_mtx_arr_from_source(cam_frames_list)
    data_joined = np.concatenate((data_robot, data_camera), -1)
    
    # 2. create joined dataset
    dataset_joined = create_dataset_shuffled(data_joined, buffer_size = data_joined.shape[0], batch_size = data_joined.shape[0]) #data_joined.shape[0])

    # 3. create variable to optimize
    var = np.array([
    [0, 0, 0, 0],
    [0, 0, 0, 0],
    [0, 0, 0, 0]
    ]).astype(np.float32)
    
    # Create the main variable with shape [3, 4]. 
    x_mtx = tf.Variable(var)

    
    #Schedule for optimizer
    lr_schedule = tf.keras.optimizers.schedules.ExponentialDecay(
        initial_learning_rate=1e-1, #1e-1,
        decay_steps=30,
        decay_rate=0.8)

    opt = tf.keras.optimizers.Adam(learning_rate=lr_schedule, amsgrad = True) # learning_rate=0.5, amsgrad = True

    # START TRAIN PROCESS 
    result_kuka_frame, loss_history, x_fr_history = train(x_mtx, loss, dataset_joined, num_epochs= 1000)

    # convert result into dict format
    result_dict = dict()
    for k, v in zip(['X','Y','Z','A','B','C'], result_kuka_frame.to_list()):
        result_dict[k] = str(v)

    return result_dict
    



    