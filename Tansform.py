import matplotlib.pyplot as plt
import numpy
import math


class Graphics:
    """定义图形的类"""

    def __init__(self):
        self.number = 0  # 顶点个数
        self.select = True  # 二、三维图形判断指标
        self.Matrix = []  # 输入坐标矩阵
        self.Matrix_proce = []  # 齐次坐标矩阵
        self.parameter = []  # 变换参数
        self.trans_matrix = numpy.array([[1, 0, 0],
                                         [0, 1, 0],
                                         [0, 0, 1]])  # 总变换矩阵
        self.matrix_x = []  # 变换矩阵
        self.matrix_finall = []  # 最终矩阵

    def run_code(self):
        """运行主程序"""
        self._project_selection()
        self._enter_point()
        self._data_process()
        self._grap_display(self.Matrix_proce)
        self._mode_select()
        print(f"变换矩阵为\n{self.trans_matrix}")
        self.matrix_finall = numpy.dot(self.Matrix_proce, self.trans_matrix)
        print(f"变换后的矩阵为\n{self.matrix_finall}")

    def _project_selection(self):
        """选择二维图形还是三维图形"""
        select_num = eval(input("请选择你需要变换的图形，二维图形输入'2'，三维图形输入'3'\n"))
        if select_num == 3:
            self.select = False
            print("三维图形支持三角体显示画面")
            self.trans_matrix = numpy.array([[1, 0, 0, 0],
                                             [0, 1, 0, 0],
                                             [0, 0, 1, 0],
                                             [0, 0, 0, 1]])  # 总变换矩阵

    def _enter_point(self):
        """输入矩阵坐标"""
        self.order = 0
        self.number = input("请输入坐标的个数\n")
        self.number = eval(self.number)
        if self.select:
            self.output_x = "[x,y]"
        else:
            self.output_x = "[x,y,z]"
        print(f"请按次序输入各点坐标,输入格式如下\n{self.output_x}")
        while self.order < self.number:
            # 依次输入各点坐标
            self.order += 1
            self.point = input(f"请输入第{self.order}个点的坐标\n")
            self.Matrix.append(eval(self.point))
        print("输入完毕")

    def _data_process(self):
        """处理Matrix_proce矩阵"""
        # 复制
        self.Matrix_proce = self.Matrix[:]
        for Matrix_point in self.Matrix_proce:
            Matrix_point.append(1)
        self.Matrix_proce = numpy.array(self.Matrix_proce)

    def _mode_select(self):
        """图形变换方式选择"""
        while True:
            print("请选择图形变换模式\n比例变换请输入'1'，对称变换请输入'2'，错切请输入'3'，旋转变换请输入'4'，平移变换请输入'5'，结束输入请输入'6'")
            num_x = eval(input())
            if num_x == 1:
                self.grap_enlarge()
            elif num_x == 2:
                self.grap_symmetry()
            elif num_x == 3:
                self.grap_miscut()
            elif num_x == 4:
                self.grap_rotate()
            elif num_x == 5:
                self.grap_move()
            elif num_x == 6:
                break
            else:
                print("错误输入")
                continue
            self._grap_display(numpy.dot(self.Matrix_proce, self.trans_matrix))

    def grap_enlarge(self):
        """比例变换"""
        if self.select:
            # 二维图形
            print("请输入x和y方向上的比例因子，格式如下")
            self.parameter = eval(input("[x,y]\n"))
            self.matrix_x = numpy.array([[self.parameter[0], 0, 0],
                                         [0, self.parameter[1], 0],
                                         [0, 0, 1]])
        else:
            # 三维图形
            print("请输入x、y和z方向上的比例因子，格式如下")
            self.parameter = eval(input("[x,y,z]\n"))
            self.matrix_x = numpy.array([[self.parameter[0], 0, 0, 0],
                                         [0, self.parameter[1], 0, 0],
                                         [0, 0, self.parameter[2], 0],
                                         [0, 0, 0, 1]])
        self.trans_matrix = numpy.dot(self.trans_matrix, self.matrix_x)

    def grap_symmetry(self):
        """对称变换"""
        if self.select:
            # 二维图形
            self.parameter = eval(input("x轴对称输入‘1’，y轴对称输入‘2’，原点对称输入‘0’\n"))
            if self.parameter == 0:
                self.matrix_x = numpy.array([[-1, 0, 0],
                                             [0, -1, 0],
                                             [0, 0, 1]])
            elif self.parameter == 1:
                self.matrix_x = numpy.array([[1, 0, 0],
                                             [0, -1, 0],
                                             [0, 0, 1]])
            elif self.parameter == 2:
                self.matrix_x = numpy.array([[-1, 0, 0],
                                             [0, 1, 0],
                                             [0, 0, 1]])
        else:
            # 三维图形
            self.parameter = eval(input("xOy平面对称输入‘1’，yOz平面对称输入‘2’，xOz平面对称输入‘3’\n"))
            if self.parameter == 1:
                self.matrix_x = numpy.array([[1, 0, 0, 0],
                                             [0, 1, 0, 0],
                                             [0, 0, -1, 0],
                                             [0, 0, 0, 1]])
            elif self.parameter == 2:
                self.matrix_x = numpy.array([[-1, 0, 0, 0],
                                             [0, 1, 0, 0],
                                             [0, 0, 1, 0],
                                             [0, 0, 0, 1]])
            elif self.parameter == 3:
                self.matrix_x = numpy.array([[1, 0, 0, 0],
                                             [0, -1, 0, 0],
                                             [0, 0, 1, 0],
                                             [0, 0, 0, 1]])

        self.trans_matrix = numpy.dot(self.trans_matrix, self.matrix_x)

    def grap_miscut(self):
        """错切变换"""
        if self.select:
            # 二维图形
            print("请输入x和y方向上的错切值，无错切则为0，格式如下")
            self.parameter = eval(input("[x,0]或[0, y]\n"))
            self.matrix_x = numpy.array([[1, self.parameter[1], 0],
                                         [self.parameter[0], 1, 0],
                                         [0, 0, 1]])
            self.trans_matrix = numpy.dot(self.trans_matrix, self.matrix_x)
        else:
            # 三维图形
            print("暂时不支持三维图形的错切")

    def grap_rotate(self):
        """旋转变换"""
        if self.select:
            # 二维图形
            rotate_value = eval(input("请输入旋转角度\n"))
            cos_x = math.cos(rotate_value / 180 * math.pi)
            sin_x = math.sin(rotate_value / 180 * math.pi)
            self.matrix_x = numpy.array([[cos_x, sin_x, 0],
                                         [-sin_x, cos_x, 0],
                                         [0, 0, 1]])
        else:
            # 三维图形
            print("请输入旋转轴和旋转角度，x轴输入‘1’，y轴输入‘2’，z轴输入‘3’，格式如下")
            self.parameter = eval(input("[对称轴，角度]\n"))
            cos_x = math.cos(self.parameter[1] / 180 * math.pi)
            sin_x = math.sin(self.parameter[1] / 180 * math.pi)
            if self.parameter == 1:
                self.matrix_x = numpy.array([[1, 0, 0, 0],
                                             [0, cos_x, sin_x, 0],
                                             [0, -sin_x, cos_x, 0],
                                             [0, 0, 0, 1]])
            if self.parameter == 2:
                self.matrix_x = numpy.array([[cos_x, 0, -sin_x, 0],
                                             [0, 1, 0, 0],
                                             [sin_x, 0, cos_x, 0],
                                             [0, 0, 0, 1]])
            if self.parameter == 3:
                self.matrix_x = numpy.array([[cos_x, sin_x, 0, 0],
                                             [-sin_x, cos_x, 0, 0],
                                             [0, 0, 1, 0],
                                             [0, 0, 0, 1]])
        self.trans_matrix = numpy.dot(self.trans_matrix, self.matrix_x)

    def grap_move(self):
        """平移变换"""
        if self.select:
            # 二维图形
            print("请输入x和y方向上的平移量，格式如下")
            self.parameter = eval(input("[x,y]\n"))
            self.matrix_x = numpy.array([[1, 0, 0],
                                         [0, 1, 0],
                                         [self.parameter[0], self.parameter[1], 1]])
        else:
            # 三维图形
            print("请输入x、y和z方向上的平移量，格式如下")
            self.parameter = eval(input("[x,y,z]\n"))
            self.matrix_x = numpy.array([[1, 0, 0, 0],
                                         [0, 1, 0, 0],
                                         [0, 0, 1, 0],
                                         [self.parameter[0], self.parameter[1], self.parameter[2], 1]])
        self.trans_matrix = numpy.dot(self.trans_matrix, self.matrix_x)

    def _grap_display(self, matrix_proce):
        """显示图像"""
        if self.select:
            # 二维图像
            axis_x = []
            axis_y = []
            for points in matrix_proce:
                axis_x.append(points[0])
                axis_y.append(points[1])
            axis_x.append(matrix_proce[0][0])
            axis_y.append(matrix_proce[0][1])
            plt.plot(axis_x, axis_y)
            plt.scatter(axis_x, axis_y)
        else:
            # 三维图像
            if self.number <= 4:
                # 定义坐标轴
                ax1 = plt.axes(projection='3d')
                axis_x = []
                axis_y = []
                axis_z = []
                for points in matrix_proce:
                    axis_x.append(points[0])
                    axis_y.append(points[1])
                    axis_z.append(points[2])
                axis_x.append(matrix_proce[0][0])
                axis_y.append(matrix_proce[0][1])
                axis_z.append(matrix_proce[0][2])
                axis_x.append(matrix_proce[2][0])
                axis_y.append(matrix_proce[2][1])
                axis_z.append(matrix_proce[2][2])
                axis_x.append(matrix_proce[1][0])
                axis_y.append(matrix_proce[1][1])
                axis_z.append(matrix_proce[1][2])
                axis_x.append(matrix_proce[3][0])
                axis_y.append(matrix_proce[3][1])
                axis_z.append(matrix_proce[3][2])
                ax1.scatter3D(axis_x, axis_y, axis_z)
                ax1.plot3D(axis_x, axis_y, axis_z)
                ax1.set_xlabel('X')  # 设置x坐标轴
                ax1.set_ylabel('Y')  # 设置y坐标轴
                ax1.set_zlabel('Z')  # 设置z坐标轴
        plt.show()


if __name__ == '__main__':
    Graphics().run_code()
