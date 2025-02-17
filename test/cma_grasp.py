from mujoco_grasping import ISFMujocoGraspingYCBObjectEvaluation
from service import generate_object_name_list

def run(object_name_list):
    grasp = ISFMujocoGraspingYCBObjectEvaluation()
    grasp.evaluate(object_name_list, isf_model="cma")


if __name__ == '__main__':

    # ----
    run(object_name_list=generate_object_name_list())
