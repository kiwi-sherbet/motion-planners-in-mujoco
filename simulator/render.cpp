<<<<<<< HEAD
=======
<<<<<<< HEAD
= sim
        self._model = self._sim.model._model
        self._data = self._sim.data._data

=======
>>>>>>> c5ed397767275c9a8902cc65799cd6f10e310dc6
// vr rendering
#include <Eigen/Dense>
#include <string>
#include <iostream>
// #include <zmqpp/zmpqq.hpp>

using Eigen::Matrix3f;
using Eigen::Vector3f;
using namespace std;

const int WIDTH = 1096 * 2;
const int HEIGHT = 1176;

/*
class CV2Renderer():
    def __init__(self, device_id, sim, cam_name, width=WIDTH, height=HEIGHT, depth=False, segmentation=False, save_path=None) -> None:

        self._render_context = MjRenderContextOffscreen(sim, device_id=device_id)
        sim.add_render_context(self._render_context)

        self._width = width
        self._height = height
        self._depth = depth
        self._segmentation = segmentation
        self._cam_id = sim.model.camera_name2id(cam_name)

        if save_path is not None:
            self._save_file = cv2.VideoWriter(save_path, cv2.VideoWriter_fourcc(*'mp4v'), 30, (width, height))
        else:
            self._save_file = None

    def render(self):
        self._render_context.render(width=self._width, height=self._height, camera_id=self._cam_id, segmentation=self._segmentation)
        img = self._render_context.read_pixels(self._width, self._height, depth=self._depth, segmentation=self._segmentation)

        if self._save_file is not None:
            self._save_file.write(img[:,::-1,::-1])
        ###
        cv2.imshow('test', img[:,::-1,::-1])
        cv2.waitKey(1)
        return img

    def close(self):
        if self._save_file is not None:
            self._save_file.release()


class VRRenderer():

    def __init__(self, socket, sim, cam_name, width=WIDTH, height=HEIGHT, depth=False, segmentation=False) -> None:

        self._socket = socket

        self._sim = sim
        self._model = self._sim.model._model
        self._data = self._sim.data._data

        self._model.vis.global_.offwidth = width
        self._model.vis.global_.offheight = height

<<<<<<< HEAD
=======
>>>>>>> 16fd651bc1ba281b148fa30448b77c58d46abdff
>>>>>>> c5ed397767275c9a8902cc65799cd6f10e310dc6
        self._cam = mujoco.MjvCamera()
        self._cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
        self._cam.fixedcamid = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_CAMERA, cam_name)

        self._glctx = mujoco.GLContext(max_width=WIDTH, max_height=HEIGHT)
<<<<<<< HEAD
        self._glctx.make_current()
=======
<<<<<<< HEAD
zzz        self._glctx.make_current()
=======
        self._glctx.make_current()
>>>>>>> 16fd651bc1ba281b148fa30448b77c58d46abdff
>>>>>>> c5ed397767275c9a8902cc65799cd6f10e310dc6

        self._sim.forward()

        self._scn = mujoco.MjvScene(self._model, maxgeom=1000)
        self._vopt = mujoco.MjvOption()
        self._pert = mujoco.MjvPerturb()
        self._con = mujoco.MjrContext(self._model, mujoco.mjtFontScale.mjFONTSCALE_150)
        self._scn.stereo = 2

        self._width = width
        self._height = height
        self._depth = depth
        self._segmentation = segmentation


    def render(self):
        mujoco.mjv_updateScene(self._model, self._data, self._vopt, self._pert, self._cam, mujoco.mjtCatBit.mjCAT_ALL, self._scn)
        # render in offscreen buffer
<<<<<<< HEAD
=======
<<<<<<< HEAD
        viewport = mujoco.MjrRect(0, 0, self._width, self._height)
        mujoco.mjr_setBuffer(mujoco.mjtFramebuffer.mjFB_OFFSCREEN, self._con)
        mujoco.mjr_render(viewport=viewport, scn=self._scn, con=self._con)
=======
>>>>>>> c5ed397767275c9a8902cc65799cd6f10e310dc6
        #render_s = time.perf_counter()
        viewport = mujoco.MjrRect(0, 0, self._width, self._height)
        mujoco.mjr_setBuffer(mujoco.mjtFramebuffer.mjFB_OFFSCREEN, self._con)
        mujoco.mjr_render(viewport=viewport, scn=self._scn, con=self._con)
        #read_s = time.perf_counter()
<<<<<<< HEAD
=======
>>>>>>> 16fd651bc1ba281b148fa30448b77c58d46abdff
>>>>>>> c5ed397767275c9a8902cc65799cd6f10e310dc6

        #img = render_context_offscreen.read_pixels(width, height, depth=depth, segmentation=segmentation)
        rgb_img = np.empty((self._height, self._width, 3), dtype=np.uint8)
        mujoco.mjr_readPixels(rgb=rgb_img, depth=None, viewport=viewport, con=self._con)
<<<<<<< HEAD
=======
<<<<<<< HEAD
        self._socket.send(rgb_img.data)
        # cv2.imshow('test_python', rgb_img[:,:,::-1])
        # cv2.waitKey(1)

        return rgb_img

*/

def getVRPose(socket):
    const int FLOAT_SIZE = 4

    socket.send(b"p")
    message = socket.recv()

    hmd_pos = np.frombuffer(message, dtype=np.float32, count=3, offset=0)
    hmd_mat = np.frombuffer(message, dtype=np.float32, count=9, offset=3 * FLOAT_SIZE).reshape((3, 3))
    left_pos = np.frombuffer(message, dtype=np.float32, count=3, offset=12 * FLOAT_SIZE)
    left_mat = np.frombuffer(message, dtype=np.float32, count=9, offset=15 * FLOAT_SIZE).reshape((3, 3))
    right_pos = np.frombuffer(message, dtype=np.float32, count=3, offset=24 * FLOAT_SIZE)
    right_mat = np.frombuffer(message, dtype=np.float32, count=9, offset=27 * FLOAT_SIZE).reshape((3, 3))
    

    local_left_pos = left_pos - hmd_pos
    local_right_pos = right_pos - hmd_pos
    mat_room2hmd = np.linalg.inv(hmd_mat)

    left_bump = np.frombuffer(message, dtype=np.float32, count=1, offset=36 * FLOAT_SIZE)
    left_button = np.frombuffer(message, dtype=np.float32, count=1, offset=37 * FLOAT_SIZE)
    left_pad = np.frombuffer(message, dtype=np.float32, count=1, offset=38 * FLOAT_SIZE)

    right_bump = np.frombuffer(message, dtype=np.float32, count=1, offset=39 * FLOAT_SIZE)
    right_button = np.frombuffer(message, dtype=np.float32, count=1, offset=40 * FLOAT_SIZE)
    right_pad = np.frombuffer(message, dtype=np.float32, count=1, offset=41 * FLOAT_SIZE)

    local_left_pos = mat_room2hmd @ local_left_pos 
    local_right_pos = mat_room2hmd @ local_right_pos

    transformed_left = -local_left_pos[[2, 0, 1]]
    transformed_left[2] = -transformed_left[2] + .2
    transformed_left[0] += .15
    transformed_right = -local_right_pos[[2, 0, 1]]
    transformed_right[0] += .15
    transformed_right[2] = -transformed_right[2] + .2

    left_orientation = mat_room2hmd @ left_mat
    right_orientation = mat_room2hmd @ right_mat

    return transformed_left, transformed_right, convert_orientation(left_orientation), convert_orientation(right_orientation), left_bump, left_button, left_pad, right_bump, right_button, right_pad


def convert_orientation(VR_orientation):
    T = np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]])
    B_inv = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]])
    return T @ np.transpose(B_inv @ VR_orientation) @ T.T


if __name__ == "__main__":
    pass
=======
>>>>>>> c5ed397767275c9a8902cc65799cd6f10e310dc6
        #read_e = time.perf_counter()
        self._socket.send(rgb_img.data)
        #cv2.imshow('test_python', rgb_img[:,:,::-1])
        #cv2.waitKey(1)
        #show_e = time.perf_counter()

        #print(f"rendering took {read_s - render_s:0.4f} seconds, reading pixels took {read_e - read_s:0.4f} seconds, while displaying it took {show_e-read_e:0.4f} seconds")

        return rgb_img

    def close(self):
        pass

*/
int getVRPose(){
    Matrix3f RIGHTFORWARD_GRIPPER;
    RIGHTFORWARD_GRIPPER << 0.0, 0.0, -1.0, 
                          0.0, 1.0, 0.0, 
                          1.0, 0.0, 0.0;


    Matrix3f RIGHTUP_GRIPPER;
    RIGHTUP_GRIPPER << 0.0, 0.0, 1.0,
               0.0, 1.0, 0.0,
               1.0, 0.0, 0.0;

    const int FLOAT_SIZE = 4;

    cout<< RIGHTFORWARD_GRIPPER<<endl;
    // const string endpoint = "tcp://*:5555";
    // zmpqq:: context context;
    // zmqpp:: socket socket(context, zmqpp::socket_type::rep);
    // socket.bind(endpoint);

    // zmqpp::message message;
    // socket.receive(message);

    float combined_buffer[42]={-0.68, 0.87, -0.36, -0.53, -0.85, -0.03, 0, -0.04, 1, -0.85, 0.53, 0.02, -0.92, 0.84, 0.27, -0.38, 0, 0.93, -0.69, 0.67, -0.28, -0.62, -0.74, -0.26, -0.71, 0.79, -0.04, 0.04,0.58, 0.81, -0.12, 0.81, -0.57, -0.99, -0.07, 0.11, 0.17, -0.2, 0.36, 0.24, -0.35, 0.02};
    // int size = zmq_recv(socket, combined_buffer,42,0);
    // if (size==-1) return -1;
    
    Vector3f hmd_pos;
    Matrix3f hmd_mat;
    Vector3f left_pos;
    Matrix3f left_mat;
    Vector3f right_pos;
    Matrix3f right_mat;

    for(int i=0; i<3; i++){
    	hmd_pos(i) = combined_buffer[i];
        left_pos(i) = combined_buffer[i+12];
        right_pos(i) = combined_buffer[i+24];
    }
    for(int i=0; i<3; i++){
        for(int j=0; j<3; j++){
            hmd_mat(i,j) = combined_buffer[i*3+j+3];
            left_mat(i,j) = combined_buffer[i*3+j+15];
            right_mat(i,j) = combined_buffer[i*3+j+27];
        }
    }
    

    cout<< "hmd_pos: "<< hmd_pos<<endl;
    cout<< "hmd_mat: "<< hmd_mat<<endl;
    cout<< "left_pos: "<< left_pos<<endl;
    cout<< "left_mat: "<< left_mat<<endl;
    cout<< "right_pos: "<< right_pos<<endl;
    cout<< "right_mat: "<< right_mat<<endl;

    Vector3f local_left_pos = left_pos - hmd_pos;
    Vector3f local_right_pos = right_pos - hmd_pos;
    Matrix3f mat_room2hmd = hmd_mat.inverse();


    float left_trigger = combined_buffer[36];
    float left_bump = combined_buffer[37];
    float left_button= combined_buffer[38];
    float left_pad= combined_buffer[39];
    
    float right_trigger = combined_buffer[40];
    float right_bump = combined_buffer[41];
    float right_button= combined_buffer[42];
    float right_pad= combined_buffer[43];

    local_left_pos = mat_room2hmd * local_left_pos; 
    local_right_pos = mat_room2hmd * local_right_pos;

    Vector3f transformed_left = -1 * local_left_pos;
    float tempValue = transformed_left(2);
    transformed_left(2) = transformed_left(1);
    transformed_left(1) = transformed_left(0);
    transformed_left(0) = tempValue;

    transformed_left(2) = -transformed_left(2) + .2;
    transformed_left(0) += .15;

    Vector3f transformed_right= -1 * local_right_pos;
    tempValue = transformed_right(2);
    transformed_right(2) = transformed_right(1);
    transformed_right(1) = transformed_right(0);
    transformed_right(0) = tempValue;

    transformed_right(2) = -transformed_right(2) + .2;
    transformed_right(0) += .15;

    Matrix3f left_orientation = mat_room2hmd * left_mat;
    Matrix3f right_orientation = mat_room2hmd * right_mat;
    


    return 1;
}

int main(){
    getVRPose();
    return 0;

}



/*
     def convert_orientation(VR_orientation):
    T = np.array([[0, 0, 1], [1, 0, 0], [0, -1, 0]])
    B_inv = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]])
    return T @ B_inv @ VR_orientation @ T.T


*/
<<<<<<< HEAD
=======
>>>>>>> 16fd651bc1ba281b148fa30448b77c58d46abdff
>>>>>>> c5ed397767275c9a8902cc65799cd6f10e310dc6
