from pymel.core import *


class StretchyJoint(object):
    def __init__(self):
        pass

    def setup(self):
        pass

    def build(self):
        pass

    @classmethod
    def from_transform(cls):
        pass


class StretchyJointSetup(StretchyJoint):
    """
    Setup class for a stretchy joint setup.

    The setup can blend between 3 types of behaviors:

    1. Aim
       The stretchy joint is always oriented towards the second control.

    2. Stretch
       The scale value of the A_JNT can be adjusted via the stretch attribute on the A_CTRL until it reaches control B.

    3. Follow
       The joints can slide between control A and B, while they maintain their current stretch and distance.
       This can be manipulated by the follow attribute on the A_CTRL
    """

    def __init__(self):
        self.guides = []

    def setup(self, custom_guides=None):

        # Creating the guides and saving them in a list for later use
        if custom_guides is None:
            self.guides.append(spaceLocator(name='A_LOC'))
            self.guides.append(spaceLocator(name='B_LOC'))
            xform(t=(0, 5, 0))

        else:
            self.guides.append(spaceLocator(name='A_LOC'))
            xform(t=custom_guides[0].translate.get())
            self.guides.append(spaceLocator(name='B_LOC'))
            xform(t=custom_guides[1].translate.get())

        for g in self.guides:
            self.set_color(g, 13)

    def build(self):

        # Creating the hierarchy
        select(clear=True)
        setup_grp = group(em=True, name='stretchyJoint_setup_GRP', w=True)
        ctrl_grp = group(em=True, name='controls_GRP', p=setup_grp)
        jnt_grp = group(em=True, name='joints_GRP', p=setup_grp)

        a_loc_t = self.guides[0].getTranslation(space='world')
        b_loc_t = self.guides[1].getTranslation(space='world')

        # Create the controls and prepare them for further use
        a_ctrl = circle(n='A_CTRL', nr=(0, 0, 1), c=(0, 0, 0), ch=False)[0]
        group(name='A_CTRL_offset_GRP', p=ctrl_grp).setTranslation(a_loc_t)
        self.set_color(a_ctrl, 14)

        b_ctrl = circle(n='B_CTRL', nr=(0, 0, 1), c=(0, 0, 0), ch=False)[0]
        group(name='B_CTRL_offset_GRP', p=ctrl_grp).setTranslation(b_loc_t)
        self.set_color(b_ctrl, 14)

        # Add the attributes to blend between different behaviors
        addAttr(a_ctrl, longName='stretch', attributeType='float', defaultValue=0, minValue=0, maxValue=1, k=True)
        addAttr(a_ctrl, longName='follow', attributeType='float', defaultValue=0, minValue=0, maxValue=1, k=True)

        select(clear=True)

        # Create and orient the joints
        a_jnt = joint(n='A_JNT', p=a_loc_t)

        b_jnt = joint(n='B_JNT', p=b_loc_t)
        joint(a_jnt, e=True, zso=True, oj='xyz', sao='yup')
        b_jnt.jointOrient.set(0, 0, 0)
        parent(a_jnt, jnt_grp, relative=True)

        # // AIM BEHAVIOR //
        # Create nodes and set up connections to get the aim constraint behavior
        # These nodes decompose the world matrices of the controls in order to get their world translations at runtime
        a_ctrl_dec_mtx = createNode('decomposeMatrix', n='A_CTRL_DEC_W_MTX')
        b_ctrl_dec_mtx = createNode('decomposeMatrix', n='B_CTRL_DEC_W_MTX')

        # Connect the world matrix with the decompose matrix node
        connectAttr(a_ctrl.worldMatrix[0], a_ctrl_dec_mtx.inputMatrix)
        connectAttr(b_ctrl.worldMatrix[0], b_ctrl_dec_mtx.inputMatrix)

        # Node to calculate the vector that points from control A to B
        a_b_ctrl_vec = createNode('plusMinusAverage', n='A_B_CTRL_vec_SUB')
        a_b_ctrl_vec.operation.set(2)

        connectAttr(b_ctrl_dec_mtx.outputTranslate, a_b_ctrl_vec.input3D[0])
        connectAttr(a_ctrl_dec_mtx.outputTranslate, a_b_ctrl_vec.input3D[1])

        # Matrix node to be composed with the vector between the controls a and b for the x-axis
        a_jnt_abs_rot_mtx = createNode('fourByFourMatrix', n='A_JNT_absolute_rot_MTX')

        # Composing the matrix with the ab-vector values
        connectAttr(a_b_ctrl_vec.output3Dx, a_jnt_abs_rot_mtx.in00)
        connectAttr(a_b_ctrl_vec.output3Dy, a_jnt_abs_rot_mtx.in01)
        connectAttr(a_b_ctrl_vec.output3Dz, a_jnt_abs_rot_mtx.in02)

        # Get the matrix for the joint orientation only to calculate the difference between the
        # joint orientation and the orientation of the vector between the controls
        a_jnt_orient_mtx = createNode('composeMatrix', n='A_JNT_orient_COM_MTX')
        a_jnt_orient_inv_mtx = createNode('inverseMatrix', n='A_JNT_orient_INV_MTX')

        connectAttr(a_jnt.jointOrient, a_jnt_orient_mtx.inputRotate)
        connectAttr(a_jnt_orient_mtx.outputMatrix, a_jnt_orient_inv_mtx.inputMatrix)

        # Nodes to calculate the relative rotation to point towards the b-control
        a_jnt_rel_rot_mtx = createNode('multMatrix', n='A_JNT_relative_rot_MTX')
        a_jnt_rot_dec_mtx = createNode('decomposeMatrix', n='A_JNT_rot_DEC_MTX')

        # Multiply the inverse joint orient matrix and the matrix that contains the absolute rotations
        connectAttr(a_jnt_abs_rot_mtx.output, a_jnt_rel_rot_mtx.matrixIn[0])
        connectAttr(a_jnt_orient_inv_mtx.outputMatrix, a_jnt_rel_rot_mtx.matrixIn[1])

        connectAttr(a_jnt_rel_rot_mtx.matrixSum, a_jnt_rot_dec_mtx.inputMatrix)

        # Connect to joint to get the final aim behavior
        connectAttr(a_jnt_rot_dec_mtx.outputRotate, a_jnt.rotate)

        # // STRETCH BEHAVIOR //
        # Create the required nodes and connections in order to achieve a stretch behavior of joint A
        # Calculate the distance between the controls and divide it by the default joint length to get a
        # stretch factor that can be piped into the scaleX value of joint A when the stretch attr is active
        distance_a_b = createNode('distanceBetween', n='distance_between_CTRLS_A_B')
        stretch_factor = createNode('multiplyDivide', n='stretch_factor_DIV')
        stretch_factor.operation.set(2)
        stretch_blend = createNode('animBlendNodeAdditiveF', n='stretch_blend')

        # Connection to get the distance
        connectAttr(a_b_ctrl_vec.output3D, distance_a_b.point2)

        # Get the stretch factor
        connectAttr(distance_a_b.distance, stretch_factor.input1X)
        stretch_factor.input2X.set(distance_a_b.distance.get())

        # Set up the blend node
        stretch_blend.inputA.set(1)
        connectAttr(stretch_factor.outputX, stretch_blend.inputB)
        stretch_attr_rev = createNode('reverse', n='stretch_attr_REV')
        connectAttr(a_ctrl.stretch, stretch_attr_rev.inputX)
        connectAttr(stretch_attr_rev.outputX, stretch_blend.weightA)
        connectAttr(a_ctrl.stretch, stretch_blend.weightB)

        # Connect with the joint to get the desired stretch behavior
        connectAttr(stretch_blend.output, a_jnt.scaleX)

        # // FOLLOW BEHAVIOR //
        # Create the nodes and connections in order to get a follow behavior of the joints
        # that allows them to dynamically follow control A or B, if the distance between the
        # controls is bigger than the one between the joints.

        # Calculate how long the the joints are apart for the case that the stretch value is
        # higher than 0
        stretch_length = createNode('multiplyDivide', n='stretch_length_MULT')
        connectAttr(stretch_factor.input2X, stretch_length.input1X)
        connectAttr(stretch_blend.output, stretch_length.input2X)

        # Difference between the joint and control distance
        jnt_ctrl_gap = createNode('plusMinusAverage', n='JNT_CTRL_gap_SUB')
        connectAttr(distance_a_b.distance, jnt_ctrl_gap.input1D[0])
        connectAttr(stretch_length.outputX, jnt_ctrl_gap.input1D[1])
        jnt_ctrl_gap.operation.set(2)

        # Clamp negative values so that joint A never moves pas control A to maintain the aim behavior
        # NOTE: I did this because I thought it would be better to maintain the aim behavior in control A,
        # if the distance between the controls is shorter than the one between the joints
        clamp_neg_val = createNode('condition', n='clamp_negative_values')
        connectAttr(jnt_ctrl_gap.output1D, clamp_neg_val.colorIfFalseR)
        connectAttr(jnt_ctrl_gap.output1D, clamp_neg_val.colorIfFalseG)
        connectAttr(jnt_ctrl_gap.output1D, clamp_neg_val.colorIfFalseB)
        connectAttr(jnt_ctrl_gap.output1D, clamp_neg_val.firstTerm)
        clamp_neg_val.operation.set(4)

        # Normalize the vector between control A and B
        norm_x_vec = createNode('vectorProduct', n='normalize_X_vec')
        connectAttr(a_b_ctrl_vec.output3D, norm_x_vec.input1)
        norm_x_vec.operation.set(0)
        norm_x_vec.normalizeOutput.set(True)

        # Calculate the vector that has the length of the gap between the joints and controls
        jnt_ctrl_gap_vec = createNode('multiplyDivide', n='JNT_CTRL_gap_vec_MULT')
        connectAttr(norm_x_vec.output, jnt_ctrl_gap_vec.input1)
        connectAttr(clamp_neg_val.outColor, jnt_ctrl_gap_vec.input2)

        # Calculate the new position of joint a in world space
        a_jnt_disp_vec = createNode('plusMinusAverage', n='A_JNT_follow_displacement_vec_ADD')
        connectAttr(a_ctrl_dec_mtx.outputTranslate, a_jnt_disp_vec.input3D[0])
        connectAttr(jnt_ctrl_gap_vec.output, a_jnt_disp_vec.input3D[1])

        # Blending the position of joint a based on the follow attribute
        follow_blend = createNode('blendColors', n='follow_blend')
        connectAttr(a_ctrl.follow, follow_blend.blender)
        connectAttr(a_jnt_disp_vec.output3D, follow_blend.color1)
        connectAttr(a_ctrl_dec_mtx.outputTranslate, follow_blend.color2)

        # Connect the translation to joint A
        connectAttr(follow_blend.output, a_jnt.translate)

        # In case the guides should be deleted from the scene
        # self.delete_guides()

    @staticmethod
    def set_color(dag_node, color_code):

        # Change the color of the shape nodes of the given object
        shapes = dag_node.getShapes()

        for s in shapes:
            s.overrideEnabled.set(1)
            s.overrideColor.set(color_code)

    @classmethod
    def from_transform(cls):

        # Create a stretchy joint with two selected transform nodes in the scene
        sel_list = (ls(selection=True, transforms=True))

        if len(sel_list) < 2:
            warning('Not enough transform nodes selected!Please select exactly two.')

        elif len(sel_list) > 2:
            warning('Too many transform nodes selected! Please select exactly two.')

        else:
            s_joint = StretchyJointSetup()

            s_joint.setup(sel_list)
            s_joint.build()

    def delete_guides(self):

        # Delete the guides
        for g in self.guides:
            delete(g)


def main():
    s_jnt = StretchyJointSetup()

    s_jnt.setup()
    s_jnt.build()


if __name__ == '__main__':
    main()
