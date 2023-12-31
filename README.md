# Registeration

---

A reappear of thr article :[Surface Registration with Eigenvalues and Eigenvectors](https://ieeexplore.ieee.org/document/8713894)

![frame](include/frame.png)

### 项目简介

复现基于特征值和特征向量的曲面配准方法，运用基于能量方程的优化计算方法使计算效果达到论文实现的效果。

### usage

```zsh 
git clone https://github.com/VIISAUS00/Registration.git
cd Registration
mkdir build
cmake ..
make
./Registeration
```



![Visualization over the first non-zero eigen vector](include/LAP.png)

### **整体代码框架**

目前程序大部分编码工作已经完成，仅差二次线性规划问题还未复现。最终代码会包装成一个Surface类。对当前编码结果进行黑盒分析，使用方法仅需构造该Surface类，输入两个Mesh格式变量，系统会自动判断两流形采样点数，边界条件等是否满足配准条件，因此目前的程序已经具有较为优秀的可靠性。调用其中的Registration()方法即可实现流形配准，通过模拟计算可知，对eight.m这一相对简单的流形进行配准，在本报告所给出的实验环境下，耗时可以控制在20分钟以内，基本达到论文所要求的运行效果。

但通过白盒分析发现，当前代码模块内部耦合性相当糟糕，各个组件之间共用一个数据存储空间，且并非通过参数形式进入模块，达到了内容耦合的程度。这是为了实现大数据传输下保证空间资源和时间资源重复占用而作出的妥协，如果其他功能对Surface类内部模块有调用需求的话，可复用程度很低。



###   Surface类重要方法及变量功能说明：  

- Surface(M* pMesh_N,M* pMesh_M,int k):构造函数，输入三个变量，前两个代表需要被配准的两个流形，最后一个代表配准中利用前k个特征值进行配准。 

- void buildLBM_N():对第一个流形构造 Laplace-Beltrami 矩阵，该方法中L_N,S_N,W_N会根据论文中的定义赋值。 

- void buildLBM_M():对第二个流形构造 Laplace-Beltrami 矩阵，该方法中L_M,S_M,W_M会根据论文中的定义赋值。  

- void EmBedding_N()：根据k计算前k个特征向量，根据论文中的定义，将利用特征向量和特征值计算出I_N;  
- void EmBedding_M():根据k计算前k个特征向量，根据论文中的定义，将利用特征向量和特征值计算出I_M;  
- void Registeration(int K);执行配准操作，进行K次迭代，每次迭代开始时会调用EmBedding_N()，根据compute所得结果迭代N的Laplace-Beltrami 矩阵特征值特征向量，随后将对compute进行下一步调用，每次迭代实现一次二次规划问题。  

- void updateOMEGA(): 该步骤会通过Registeration被调用，将一个对角矩阵更新为具有位置对应信息的矩阵以实现配准效果。  

- void compute()：实现二次规划，目前还在开发设计，难点在于处理大型稀疏矩阵，广泛使用的二次规划问题解决方法都涉及到参数矩阵求逆，而在此处稀疏大矩阵所求逆矩阵往往不可逆，因此需要重新查找可复用的计算方法并复现。  