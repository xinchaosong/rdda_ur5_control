import socket
import ast


class ControlClientPy3:
    """
    This class is for controlling UR5 robotic arm with RDDA.
    """

    def __init__(self):
        self.udp_recv_port = 56801
        self.udp_sent_port = 56800
        self.udp_client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.udp_client.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.udp_client.bind(("", self.udp_recv_port))

    def set_rdda_stiffness(self, stiff0, stiff1):
        command_str = 'set_rdda_stiffness,%f,%f' % (stiff0, stiff1)
        self.udp_client.sendto(command_str.encode(), ('<broadcast>', self.udp_sent_port))
        data, addr = self.udp_client.recvfrom(16)

        return data.decode()

    def init_rdda_stiffness(self):
        command_str = 'init_rdda_stiffness'
        self.udp_client.sendto(command_str.encode(), ('<broadcast>', self.udp_sent_port))
        data, addr = self.udp_client.recvfrom(16)

        return data.decode()

    def set_rdda_positions(self, position0, position1):
        command_str = 'set_rdda_positions,%f,%f' % (position0, position1)
        self.udp_client.sendto(command_str.encode(), ('<broadcast>', self.udp_sent_port))
        data, addr = self.udp_client.recvfrom(16)

        return data.decode()

    def set_rdda_max_velocities(self, max_velocity0, max_velocity1):
        command_str = 'set_rdda_max_velocities,%f,%f' % (max_velocity0, max_velocity1)
        self.udp_client.sendto(command_str.encode(), ('<broadcast>', self.udp_sent_port))
        data, addr = self.udp_client.recvfrom(16)

        return data.decode()

    def set_rdda_max_efforts(self, max_effort0, max_effort1):
        command_str = 'set_rdda_max_efforts,%f,%f' % (max_effort0, max_effort1)
        self.udp_client.sendto(command_str.encode(), ('<broadcast>', self.udp_sent_port))
        data, addr = self.udp_client.recvfrom(16)

        return data.decode()

    def home_rdda(self):
        command_str = 'home_rdda'
        self.udp_client.sendto(command_str.encode(), ('<broadcast>', self.udp_sent_port))
        data, addr = self.udp_client.recvfrom(16)

        return data.decode()

    def read_rdda_positions(self):
        command_str = 'read_rdda_positions'
        self.udp_client.sendto(command_str.encode(), ('<broadcast>', self.udp_sent_port))
        data, addr = self.udp_client.recvfrom(64)

        return ast.literal_eval(data.decode())

    def read_rdda_lower_bounds(self):
        command_str = 'read_rdda_lower_bounds'
        self.udp_client.sendto(command_str.encode(), ('<broadcast>', self.udp_sent_port))
        data, addr = self.udp_client.recvfrom(64)

        return ast.literal_eval(data.decode())

    def read_rdda_upper_bounds(self):
        command_str = 'read_rdda_upper_bounds'
        self.udp_client.sendto(command_str.encode(), ('<broadcast>', self.udp_sent_port))
        data, addr = self.udp_client.recvfrom(64)

        return ast.literal_eval(data.decode())

    def read_rdda_origins(self):
        command_str = 'read_rdda_origins'
        self.udp_client.sendto(command_str.encode(), ('<broadcast>', self.udp_sent_port))
        data, addr = self.udp_client.recvfrom(64)

        return ast.literal_eval(data.decode())

    def read_ur5_position(self):
        command_str = 'read_ur5_position'
        self.udp_client.sendto(command_str.encode(), ('<broadcast>', self.udp_sent_port))
        data, addr = self.udp_client.recvfrom(128)

        return ast.literal_eval(data.decode())

    def move_ur5(self, x, y, z, roll, pitch, yaw, velocity):
        command_str = 'move_ur5,%f,%f,%f,%f,%f,%f,%f' % (x, y, z, roll, pitch, yaw, velocity)
        self.udp_client.sendto(command_str.encode(), ('<broadcast>', self.udp_sent_port))
        data, addr = self.udp_client.recvfrom(16)

        return data.decode()

    def move_ur5_linear(self, axis, target, velocity, wait=True):
        command_str = 'move_ur5_linear,%d,%f,%f,%d' % (axis, target, velocity, wait)
        self.udp_client.sendto(command_str.encode(), ('<broadcast>', self.udp_sent_port))
        data, addr = self.udp_client.recvfrom(16)

        return data.decode()

    def stop_ur5(self):
        command_str = 'stop_ur5'
        self.udp_client.sendto(command_str.encode(), ('<broadcast>', self.udp_sent_port))
        data, addr = self.udp_client.recvfrom(16)

        return data.decode()

    def home_ur5(self):
        command_str = 'home_ur5'
        self.udp_client.sendto(command_str.encode(), ('<broadcast>', self.udp_sent_port))
        data, addr = self.udp_client.recvfrom(16)

        return data.decode()

    def move_read_discrete(self, step_size, step_num):
        command_str = 'move_read_discrete,%f,%d' % (step_size, step_num)
        self.udp_client.sendto(command_str.encode(), ('<broadcast>', self.udp_sent_port))
        data, addr = self.udp_client.recvfrom(10240)

        return ast.literal_eval(data.decode())

    def move_read_continuous(self, step_size, step_num):
        command_str = 'move_read_continuous,%f,%d' % (step_size, step_num)
        self.udp_client.sendto(command_str.encode(), ('<broadcast>', self.udp_sent_port))
        data, addr = self.udp_client.recvfrom(10240)

        return ast.literal_eval(data.decode())
