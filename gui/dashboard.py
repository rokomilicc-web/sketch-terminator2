import os
import time
import streamlit as st
import rclpy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration

from kinematics import Kinematics

# Set page config
st.set_page_config(
    page_title="Sketch Terminator Standalone Kinematics Tester",
    page_icon="🤖",
    layout="wide",
    initial_sidebar_state="expanded"
)

# Custom Premium CSS Styling (Glassmorphism & Cyber-Dark UI)
st.markdown("""
<style>
    @import url('https://fonts.googleapis.com/css2?family=Orbitron:wght@400;700&family=Outfit:wght@300;400;600&display=swap');
    
    html, body, [class*="css"] {
        font-family: 'Outfit', sans-serif;
        background-color: #0c0f1d;
        color: #e2e8f0;
    }
    
    .stApp {
        background: radial-gradient(circle at 50% 50%, #151a30 0%, #080a12 100%);
    }

    h1, h2, h3, h4 {
        font-family: 'Orbitron', sans-serif !important;
        letter-spacing: 2px;
        color: #00f2fe;
        text-shadow: 0 0 10px rgba(0, 242, 254, 0.4);
    }
    
    /* Premium Glassmorphic Cards applied to Streamlit's container borders */
    div[data-testid="stVerticalBlockBorderWrapper"] {
        background: rgba(21, 26, 48, 0.45) !important;
        backdrop-filter: blur(12px) !important;
        -webkit-backdrop-filter: blur(12px) !important;
        border: 1px solid rgba(255, 255, 255, 0.08) !important;
        border-radius: 16px !important;
        padding: 24px !important;
        margin-bottom: 24px !important;
        box-shadow: 0 8px 32px 0 rgba(0, 0, 0, 0.37) !important;
    }
    
    /* Subtitle glow */
    .glow-text {
        color: #4facfe;
        text-shadow: 0 0 8px rgba(79, 172, 254, 0.4);
        font-family: 'Orbitron', sans-serif;
    }
</style>
""", unsafe_allow_html=True)

# Initialize Session State
if 'kinematics' not in st.session_state:
    st.session_state.kinematics = Kinematics()

# Helper function to sync GUI session state values back to widget keys
def sync_widget_states():
    st.session_state.j1_slider = float(st.session_state.j1_val)
    st.session_state.j1_text = f"{st.session_state.j1_val:.3f}"
    st.session_state.j2_slider = float(st.session_state.j2_val)
    st.session_state.j2_text = f"{st.session_state.j2_val:.3f}"
    st.session_state.j3_slider = float(st.session_state.j3_val)
    st.session_state.j3_text = f"{st.session_state.j3_val:.3f}"
    
    st.session_state.ik_x_slider = float(st.session_state.ik_x_val)
    st.session_state.ik_x_text = f"{st.session_state.ik_x_val:.3f}"
    st.session_state.ik_y_slider = float(st.session_state.ik_y_val)
    st.session_state.ik_y_text = f"{st.session_state.ik_y_val:.3f}"
    st.session_state.ik_z_slider = float(st.session_state.ik_z_val)
    st.session_state.ik_z_text = f"{st.session_state.ik_z_val:.3f}"

# Function to fetch current joints from ROS 2 on startup
def get_current_joints(node):
    class JointStatesSubscriber:
        def __init__(self):
            self.received = False
            self.positions = [0.0, 0.0, 0.0]
        def callback(self, msg):
            name_map = {n: i for i, n in enumerate(msg.name)}
            if 'joint1' in name_map and 'joint2' in name_map and 'joint3' in name_map:
                self.positions = [
                    msg.position[name_map['joint1']],
                    msg.position[name_map['joint2']],
                    msg.position[name_map['joint3']]
                ]
                self.received = True

    sub_obj = JointStatesSubscriber()
    sub = node.create_subscription(JointState, '/joint_states', sub_obj.callback, 10)
    
    # Spin up to 1 second
    start_time = time.time()
    while not sub_obj.received and (time.time() - start_time) < 1.0:
        rclpy.spin_once(node, timeout_sec=0.05)
        
    node.destroy_subscription(sub)
    return sub_obj.positions if sub_obj.received else [0.0, 0.0, 0.0]

# Initialize ROS 2 Node in session state
if 'ros_node' not in st.session_state:
    if not rclpy.ok():
        rclpy.init()
    node = rclpy.create_node('streamlit_gui_node')
    st.session_state.ros_node = node
    st.session_state.joint_pub = node.create_publisher(
        JointTrajectory,
        '/joint_trajectory_controller/joint_trajectory',
        10
    )

# Initialize default values
if 'j1_val' not in st.session_state:
    # Fetch current joint states from hardware
    init_joints = [0.0, 0.0, 0.0]
    try:
        init_joints = get_current_joints(st.session_state.ros_node)
    except Exception:
        pass
    
    # Order: [joint1 (beta), joint2 (gama), joint3 (alpha)]
    st.session_state.j1_val = float(init_joints[2]) # alpha
    st.session_state.j2_val = float(init_joints[0]) # beta
    st.session_state.j3_val = float(init_joints[1]) # gama

    # Calculate initial Cartesian position
    x, y, z = st.session_state.kinematics.get_dk(st.session_state.j2_val, st.session_state.j3_val, st.session_state.j1_val)
    st.session_state.ik_x_val = float(x)
    st.session_state.ik_y_val = float(y)
    st.session_state.ik_z_val = float(z)
    
    sync_widget_states()

# Sidebar layout
st.sidebar.markdown("<h1 style='text-align: center; font-size: 20px; color: #00f2fe; text-shadow: 0 0 10px rgba(0, 242, 254, 0.4);'>STANDALONE KINEMATICS</h1>", unsafe_allow_html=True)
st.sidebar.markdown("---")

st.sidebar.markdown("### 🦾 Joint Status")
st.sidebar.markdown(f"- **J1 (Base - Alpha):** `{st.session_state.j1_val:.3f} rad`")
st.sidebar.markdown(f"- **J2 (Shoulder - Beta):** `{st.session_state.j2_val:.3f} rad`")
st.sidebar.markdown(f"- **J3 (Elbow - Gama):** `{st.session_state.j3_val:.3f} rad`")
st.sidebar.info("ROS 2 is ACTIVE. GUI is publishing to `/joint_trajectory_controller/joint_trajectory`.")

def publish_joints(j1, j2, j3):
    try:
        msg = JointTrajectory()
        msg.joint_names = ['joint1', 'joint2', 'joint3']
        point = JointTrajectoryPoint()
        point.positions = [float(j2), float(j3), float(j1)]
        point.time_from_start = Duration(sec=0, nanosec=150000000) # 150ms for smooth tracking
        msg.points.append(point)
        st.session_state.joint_pub.publish(msg)
    except Exception:
        pass

def on_joint_change():
    # Sync IK values from joints
    x, y, z = st.session_state.kinematics.get_dk(st.session_state.j2_val, st.session_state.j3_val, st.session_state.j1_val)
    st.session_state.ik_x_val = float(x)
    st.session_state.ik_y_val = float(y)
    st.session_state.ik_z_val = float(z)
    publish_joints(st.session_state.j1_val, st.session_state.j2_val, st.session_state.j3_val)

def on_ik_change():
    # Solve IK from XYZ
    try:
        b_val, g_val, a_val = st.session_state.kinematics.get_ik(st.session_state.ik_x_val, st.session_state.ik_y_val, st.session_state.ik_z_val)
        st.session_state.j1_val = float(a_val)
        st.session_state.j2_val = float(b_val)
        st.session_state.j3_val = float(g_val)
        publish_joints(a_val, b_val, g_val)
    except Exception:
        pass # Out of reach

# Bidirectional sync callbacks
def update_j1_slider():
    st.session_state.j1_val = st.session_state.j1_slider
    on_joint_change()
    sync_widget_states()

def update_j1_text():
    try:
        val = float(st.session_state.j1_text)
        st.session_state.j1_val = max(-3.14, min(3.14, val))
    except ValueError:
        pass
    on_joint_change()
    sync_widget_states()

def update_j2_slider():
    st.session_state.j2_val = st.session_state.j2_slider
    on_joint_change()
    sync_widget_states()

def update_j2_text():
    try:
        val = float(st.session_state.j2_text)
        st.session_state.j2_val = max(-3.14, min(3.14, val))
    except ValueError:
        pass
    on_joint_change()
    sync_widget_states()

def update_j3_slider():
    st.session_state.j3_val = st.session_state.j3_slider
    on_joint_change()
    sync_widget_states()

def update_j3_text():
    try:
        val = float(st.session_state.j3_text)
        st.session_state.j3_val = max(-3.14, min(3.14, val))
    except ValueError:
        pass
    on_joint_change()
    sync_widget_states()

def update_ik_x_slider():
    st.session_state.ik_x_val = st.session_state.ik_x_slider
    on_ik_change()
    sync_widget_states()

def update_ik_x_text():
    try:
        val = float(st.session_state.ik_x_text)
        st.session_state.ik_x_val = max(-0.45, min(0.45, val))
    except ValueError:
        pass
    on_ik_change()
    sync_widget_states()

def update_ik_y_slider():
    st.session_state.ik_y_val = st.session_state.ik_y_slider
    on_ik_change()
    sync_widget_states()

def update_ik_y_text():
    try:
        val = float(st.session_state.ik_y_text)
        st.session_state.ik_y_val = max(-0.45, min(0.45, val))
    except ValueError:
        pass
    on_ik_change()
    sync_widget_states()

def update_ik_z_slider():
    st.session_state.ik_z_val = st.session_state.ik_z_slider
    on_ik_change()
    sync_widget_states()

def update_ik_z_text():
    try:
        val = float(st.session_state.ik_z_text)
        st.session_state.ik_z_val = max(-0.05, min(0.20, val))
    except ValueError:
        pass
    on_ik_change()
    sync_widget_states()


col1, col2 = st.columns(2)

with col1:
    with st.container(border=True):
        st.markdown("<h2>Direct Kinematics</h2>", unsafe_allow_html=True)
        st.markdown("<p style='color:#a0aec0;'>Test Joint-to-Cartesian (DK) calculation.</p>", unsafe_allow_html=True)
        
        # Row 1: Joint 1
        st.markdown("#### Joint 1 (Base/Alpha) [rad]")
        c_sld, c_txt = st.columns([3, 1])
        c_sld.slider("j1_sld_lbl", -3.14, 3.14, key="j1_slider", on_change=update_j1_slider, label_visibility="collapsed", step=0.01)
        c_txt.text_input("j1_txt_lbl", key="j1_text", on_change=update_j1_text, label_visibility="collapsed")

        # Row 2: Joint 2
        st.markdown("#### Joint 2 (Shoulder/Beta) [rad]")
        c_sld, c_txt = st.columns([3, 1])
        c_sld.slider("j2_sld_lbl", -3.14, 3.14, key="j2_slider", on_change=update_j2_slider, label_visibility="collapsed", step=0.01)
        c_txt.text_input("j2_txt_lbl", key="j2_text", on_change=update_j2_text, label_visibility="collapsed")

        # Row 3: Joint 3
        st.markdown("#### Joint 3 (Elbow/Gama) [rad]")
        c_sld, c_txt = st.columns([3, 1])
        c_sld.slider("j3_sld_lbl", -3.14, 3.14, key="j3_slider", on_change=update_j3_slider, label_visibility="collapsed", step=0.01)
        c_txt.text_input("j3_txt_lbl", key="j3_text", on_change=update_j3_text, label_visibility="collapsed")

        st.markdown("<h3 class='glow-text'>Computed Cartesian Pose:</h3>", unsafe_allow_html=True)
        # DK calculations
        x_dk, y_dk, z_dk = st.session_state.kinematics.get_dk(st.session_state.j2_val, st.session_state.j3_val, st.session_state.j1_val)
        st.code(f"X: {x_dk*1000.0:.1f} mm  ({x_dk:.4f} m)\n"
                f"Y: {y_dk*1000.0:.1f} mm  ({y_dk:.4f} m)\n"
                f"Z: {z_dk*1000.0:.1f} mm  ({z_dk:.4f} m)")

with col2:
    with st.container(border=True):
        st.markdown("<h2>Inverse Kinematics</h2>", unsafe_allow_html=True)
        st.markdown("<p style='color:#a0aec0;'>Test Cartesian-to-Joint (IK) calculation.</p>", unsafe_allow_html=True)

        # Row 1: X Position
        st.markdown("#### X Position [m]")
        c_sld, c_txt = st.columns([3, 1])
        c_sld.slider("ik_x_sld_lbl", -0.45, 0.45, key="ik_x_slider", on_change=update_ik_x_slider, label_visibility="collapsed", step=0.005)
        c_txt.text_input("ik_x_txt_lbl", key="ik_x_text", on_change=update_ik_x_text, label_visibility="collapsed")

        # Row 2: Y Position
        st.markdown("#### Y Position [m]")
        c_sld, c_txt = st.columns([3, 1])
        c_sld.slider("ik_y_sld_lbl", -0.45, 0.45, key="ik_y_slider", on_change=update_ik_y_slider, label_visibility="collapsed", step=0.005)
        c_txt.text_input("ik_y_txt_lbl", key="ik_y_text", on_change=update_ik_y_text, label_visibility="collapsed")

        # Row 3: Z Position
        st.markdown("#### Z Position [m]")
        c_sld, c_txt = st.columns([3, 1])
        c_sld.slider("ik_z_sld_lbl", -0.05, 0.20, key="ik_z_slider", on_change=update_ik_z_slider, label_visibility="collapsed", step=0.005)
        c_txt.text_input("ik_z_txt_lbl", key="ik_z_text", on_change=update_ik_z_text, label_visibility="collapsed")

        # Calculate IK for display status
        try:
            b_val, g_val, a_val = st.session_state.kinematics.get_ik(st.session_state.ik_x_val, st.session_state.ik_y_val, st.session_state.ik_z_val)
            ik_success = True
        except Exception:
            ik_success = False

        st.markdown("<h3 class='glow-text'>Computed Joint Values:</h3>", unsafe_allow_html=True)
        if ik_success:
            st.code(f"J1 (Alpha): {a_val:.4f} rad ({a_val * 180.0 / 3.1415:.1f}°)\n"
                    f"J2 (Beta):  {b_val:.4f} rad ({b_val * 180.0 / 3.1415:.1f}°)\n"
                    f"J3 (Gama):  {g_val:.4f} rad ({g_val * 180.0 / 3.1415:.1f}°)")
        else:
            st.error("IK Resolution Failed: Target is out of reach or mathematically invalid.")
