/**
 * @brief:Includes
 */
#include "minimumjerk.h"
#include <iostream>
#include <cmath>
#include <tf/tf.h>

/**
 * @brief:MinimumJerk::MinimumJerk()
 */
MinimumJerk::MinimumJerk()
{
    // 轨迹偏离原点大小
    origin_x_ = 0;
    origin_y_ = 0;
    origin_z_ = 0;
}
void MinimumJerk::solution(Eigen::MatrixXd &path_points_3d, Eigen::MatrixXd &T_allocation)
{
    // 查看合法性，时间分配和轨迹段是否相同，每一段轨迹有其相应的时间分配
    if (path_points_3d.rows() - 1 != T_allocation.rows()) //不合法
    {
        std::cout << "wrongful" << std::endl;
        return;
    }
    //合法，执行以下部分
    path_points_3d_ = path_points_3d;
    a_path_t = 2.5;
    // 时间分配
    closed_form_solution_3D_with_time_allocation(path_points_3d, T_allocation, a_xyz_); //传点求解多项式曲线
    get_waypoints(T_allocation);                                                        //获取点
    publish_waypoints();                                                                //发布点
}

/**
 * @brief MinimumSnap::compute_Q
 * @param n 多项式阶次
 * @param r:求导阶数
 *        r=1 --- minimum vel
 *        r=2 --- minimum acc
 *        r=3 --- minimum jerk
 *        r=4 --- minimum snap
 * @param t1 前一时刻
 * @param t2 当前时刻
 * @return
 */
//建立Qi
Eigen::MatrixXd MinimumJerk::compute_Q(int n, int r, float t1, float t2) //n阶，r求导阶数,，t1，t2时间
{
    //求出（1+t+t^2+t^3+...+t^n）的个数
    int temp_val = (n - r) * 2 + 1;
    Eigen::MatrixXd T(temp_val, 1);
    for (int i = 1; i <= temp_val; i++)
    {
        // pow(x, y)   求x的y次幂
        T(i - 1, 0) = pow(t2, i) - pow(t1, i);
    }

    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(n + 1, n + 1);

    for (int i = r + 1; i <= n + 1; i++)
    {
        for (int j = i; j <= n + 1; j++)
        {
            int k1 = i - r - 1;
            int k2 = j - r - 1;
            int k = k1 + k2 + 1;
            Eigen::MatrixXd m_1(r, 1);
            Eigen::MatrixXd m_2(r, 1);

            for (int a = 1; a <= r; a++)
            {
                m_1(a - 1, 0) = k1 + a;
                m_2(a - 1, 0) = k2 + a;
            }

            Q(i - 1, j - 1) = m_1.prod() * m_2.prod() / k * T(k - 1, 0);
            Q(j - 1, i - 1) = Q(i - 1, j - 1);
        }
    }
    return Q;
}

/**
 * @brief set_matrix_diagonal_block
 * @param result
 * @param block
 * @param block_pos
 * @return
 */
//插入对角矩阵
bool MinimumJerk::insert_matrix_diagonal_block(Eigen::MatrixXd &result, Eigen::MatrixXd &block, const int block_pos)
{
    int block_size = block.rows();
    int pos = (block_pos - 1) * block.rows();
    result.block(pos, pos, block_size, block_size) = block;
    return true;
}

/**
 * @brief MinimumSnap::compute_C_transpose
 * @param n 多项式段数
 */
Eigen::MatrixXd MinimumJerk::compute_C_transpose(int n)
{
    // 行大小
    int row_size;
    // 列大小
    int col_size;

    // compute the size of row and col
    row_size = 6 * n;
    col_size = 6 * n - 3 * (n - 1);

    // create C_transpose Matrix
    Eigen::MatrixXd C_transpose = Eigen::MatrixXd::Zero(row_size, col_size);

    // first p
    C_transpose(0, 0) = 1;
    C_transpose(1, 1) = 1;
    C_transpose(2, 2) = 1;
    //C_transpose(3 , 3) = 1;

    // last p
    C_transpose(row_size - 3, 2 + (n - 1) + 1) = 1;
    C_transpose(row_size - 2, 2 + (n - 1) + 2) = 1;
    C_transpose(row_size - 1, 2 + (n - 1) + 3) = 1;

    // middle p
    // i = n - 1  首末 4+4=8 已经赋值了
    for (int i = 0; i < n - 1; i++)
    {
        C_transpose(2 + 6 * i + 1, 3 + i) = 1;
        C_transpose(2 + 6 * i + 2, i * 2 + 6 + (n - 1)) = 1;
        C_transpose(2 + 6 * i + 3, i * 2 + 6 + (n - 1) + 1) = 1;

        C_transpose(2 + 6 * i + 4, 3 + i) = 1;
        C_transpose(2 + 6 * i + 5, i * 2 + 6 + (n - 1)) = 1;
        C_transpose(2 + 6 * i + 6, i * 2 + 6 + (n - 1) + 1) = 1;
    }
    return C_transpose;
}

/**
 * @brief MinimumSnap::closed_form_solution
 *        多项式阶次  7次多项式，8个系数
 *
 * @param waypoints
 * @param result
 * @return
 */
bool MinimumJerk::closed_form_solution(Eigen::MatrixXd &waypoints, Eigen::MatrixXd &T_allocation, Eigen::MatrixXd &result)
{
    // 获取waypoint 个数
    int waypoints_num = waypoints.rows();
    // 轨迹的段数
    int segments_num = waypoints_num - 1;
    // 时间分配
    float T = 2.5;
    // 多项式阶次
    int segment_order = 5;
    // minimum set
    int minimum_set = 3;

    // 个矩阵行列大小
    int Qi_rows = 6;
    int Qi_cols = 6;
    int Q_rows = 6 * segments_num;
    int Q_cols = 6 * segments_num;

    //系数
    int a_rows = 6 * segments_num;
    int a_cols = 1;

    int Mi_rows = 6;
    int Mi_cols = 6;
    int M_rows = 6 * segments_num;
    int M_cols = 6 * segments_num;

    int C_transpose_rows = 6 * segments_num;
    int C_transpose_cols = 6 * segments_num - 3 * (segments_num - 1); //
    int C_rows = 6 * segments_num - 3 * (segments_num - 1);           //
    int C_cols = 6 * segments_num;

    int d_rows = 6 * segments_num;
    int d_cols = 1;

    int dfp_rows = 3 * segments_num + 3;
    int dfp_cols = 1;
    int df_rows = 6 + segments_num - 1;
    int df_cols = 1;
    int dp_rows = 2 * (segments_num - 1);
    int dp_cols = 1;

    int R_rows = 3 * segments_num + 3;
    int R_cols = 3 * segments_num + 3;

    int R_FF_rows = df_rows;
    int R_FF_cols = df_rows;
    int R_FP_rows = df_rows;
    int R_FP_cols = dp_rows;
    int R_PF_rows = dp_rows;
    int R_PF_cols = df_rows;
    int R_PP_rows = dp_rows;
    int R_PP_cols = dp_rows;

    /**
     * @brief 多项式系数(多段合成)
     */
    Eigen::MatrixXd a = Eigen::MatrixXd::Zero(a_rows, 1);

    /**
     * @brief 计算Qi(每个多相似对应一个), 并计算大Q 由Qi对角组成
     *        Qi形式都相同，不同之处在于 T时间的分配
     */
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(Q_rows, Q_cols);

    for (int i = 0; i < segments_num; i++)
    {
        Eigen::MatrixXd Q_i = compute_Q(segment_order, minimum_set, 0, T);
        //Q = set_matrix_diagonal_block(Q, Q_i, i + 1);
        insert_matrix_diagonal_block(Q, Q_i, i + 1);
    }
    /**
     * @brief 计算Mi,Mi的形式相似，只有T时间分配可能有区别，计算大M
     */
    Eigen::MatrixXd M = Eigen::MatrixXd::Zero(M_rows, M_cols);
    Eigen::MatrixXd M_inverse = Eigen::MatrixXd::Zero(M_rows, M_cols);
    Eigen::MatrixXd M_inverse_transpose = Eigen::MatrixXd::Zero(M_rows, M_cols);

    for (int i = 0; i < segments_num; i++)
    {
        Eigen::MatrixXd Mi = Eigen::MatrixXd::Zero(Mi_rows, Mi_cols);
        Mi << 1, 0, 0, 0, 0, 0,
            0, 1, 0, 0, 0, 0,
            0, 0, 2, 0, 0, 0,
            1, T, pow(T, 2), pow(T, 3), pow(T, 4), pow(T, 5),
            0, 1, 2 * T, 3 * pow(T, 2), 4 * pow(T, 3), 5 * pow(T, 4),
            0, 0, 2, 6 * T, 12 * pow(T, 2), 20 * pow(T, 3);
        //M = set_matrix_diagonal_block(M, Mi, i + 1);
        insert_matrix_diagonal_block(M, Mi, i + 1);
    }
    // 计算
    M_inverse = M.inverse();
    M_inverse_transpose = M_inverse.transpose();

    /**
     * @brief 计算C_transpose
     */
    Eigen::MatrixXd C_transpose = compute_C_transpose(segments_num);
    Eigen::MatrixXd C = C_transpose.transpose();

    /**
     * @brief 计算d df， dp
     */
    Eigen::MatrixXd dfp = Eigen::MatrixXd::Zero(dfp_rows, dfp_cols);
    Eigen::MatrixXd df = Eigen::MatrixXd::Zero(df_rows, dfp_cols);
    Eigen::MatrixXd dp = Eigen::MatrixXd::Zero(dp_rows, dfp_cols);

    df(0, 0) = waypoints(0, 0); // 起始点位置约束（已知）
    df(1, 0) = 3;               // 起始点acc约束（已知）
    df(2, 0) = 0;               // 起始点jerk约束（已知）
    //df(3, 0) = 0;    // 起始点snap约束（已知）

    for (int i = 3; i < (df_rows - 3); i++)
    {
        df(i, 0) = waypoints(i - 3 + 1, 0);
    }

    df(df_rows - 3, 0) = waypoints(waypoints_num - 1, 0); // 末端点位置约束（已知）
    df(df_rows - 2, 0) = 3;                               // 末端点v约束（已知）
    df(df_rows - 1, 0) = 0;                               // 末端点acc约束（已知）

    /**
     * @brief 计算R = C * A_inverse_transpose * Q * A_inverse * C_transpose
     *    R: 8 * segments_num - 4 * (n - 1) X 8 * segments_num - 4 * (n - 1)
     */
    Eigen::MatrixXd R = C * M_inverse_transpose * Q * M_inverse * C_transpose;
    /**
     * @brief 获取 R_FF  R_FP  R_PF  R_PP
     *            R_FP_transpose
     *            R_PP_inverse
     */
    Eigen::MatrixXd R_FF = R.topLeftCorner(df_rows, df_rows);
    Eigen::MatrixXd R_FP = R.topRightCorner(df_rows, dp_rows);
    Eigen::MatrixXd R_PF = R.bottomLeftCorner(dp_rows, df_rows);
    Eigen::MatrixXd R_PP = R.bottomRightCorner(dp_rows, dp_rows);
    Eigen::MatrixXd R_FP_transpose = R_FP.transpose();
    Eigen::MatrixXd R_PP_inverse = R_PP.inverse();

    /**
     * @brief 求解dp
     */
    dp = -R_PP_inverse * R_FP_transpose * df;
    for (int i = 0; i < df_rows; i++)
    {
        dfp(i, 0) = df(i, 0);
    }

    for (int i = 0; i < dp_rows; i++)
    {
        dfp(df_rows + i, 0) = dp(i, 0);
    }

    /**
     * @brief 求解d   d=M*a a=M_inverse*d
     *               d= C_transpose * dfp;
     */

    Eigen::MatrixXd d = C_transpose * dfp;

    /**
     * @brief 求解系数a
     */
    result = M_inverse * d;
}

/**
 * @brief MinimumSnap::closed_form_solution
 *        多项式阶次  7次多项式，8个系数
 *
 * @param waypoints
 * @param result
 * @return
 */
bool MinimumJerk::closed_form_solution_3D_with_time_allocation(Eigen::MatrixXd &waypoints_xyz, Eigen::MatrixXd &T_allocation, Eigen::MatrixXd &result)
{
    // 检查入口变量的合法性，防止出错
    if (waypoints_xyz.cols() != 3)
    {
        // std::cout<<"waypoints_xyz.cols() != 3  "<<std::endl;
        return false;
    }

    // 至少有两个路径点，否则，不执行
    if (waypoints_xyz.size() < 2 * 3)
    {
        return false;
    }

    //至少一段轨迹，一个T
    if (T_allocation.size() < 1)
    {
        return false;
    }

    // 获取waypoint 个数
    int waypoints_num = waypoints_xyz.rows();
    // 轨迹的段数
    int segments_num = waypoints_num - 1;
    // 时间分配
    float T;
    // 多项式阶次
    int segment_order = 5;
    // minimum set
    int minimum_set = 3;

    // 个矩阵行列大小
    int Qi_rows = 6;
    int Qi_cols = 6;
    int Q_rows = 6 * segments_num;
    int Q_cols = 6 * segments_num;

    //系数
    int a_rows = 6 * segments_num;
    int a_cols = 1;

    int Mi_rows = 6;
    int Mi_cols = 6;
    int M_rows = 6 * segments_num;
    int M_cols = 6 * segments_num;

    int C_transpose_rows = 6 * segments_num;
    int C_transpose_cols = 6 * segments_num - 3 * (segments_num - 1); //
    int C_rows = 6 * segments_num - 3 * (segments_num - 1);           //
    int C_cols = 6 * segments_num;

    int d_rows = 6 * segments_num;
    int d_cols = 1;

    int dfp_rows = 3 * segments_num + 3;
    int dfp_cols = 1;
    int df_rows = 6 + segments_num - 1;
    int df_cols = 1;
    int dp_rows = 2 * (segments_num - 1);
    int dp_cols = 1;

    int R_rows = 3 * segments_num + 3;
    int R_cols = 3 * segments_num + 3;

    int R_FF_rows = df_rows;
    int R_FF_cols = df_rows;
    int R_FP_rows = df_rows;
    int R_FP_cols = dp_rows;
    int R_PF_rows = dp_rows;
    int R_PF_cols = df_rows;
    int R_PP_rows = dp_rows;
    int R_PP_cols = dp_rows;

    /**
     * @brief 多项式系数(多段合成)
     */
    Eigen::MatrixXd a = Eigen::MatrixXd::Zero(a_rows, 1);

    /**
     * @brief 计算Qi(每个多相似对应一个), 并计算大Q 由Qi对角组成
     *        Qi形式都相同，不同之处在于 T时间的分配
     */
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(Q_rows, Q_cols);

    for (int i = 0; i < segments_num; i++)
    {
        T = T_allocation(i, 0);
        Eigen::MatrixXd Q_i = compute_Q(segment_order, minimum_set, 0, T);
        //Q = set_matrix_diagonal_block(Q, Q_i, i + 1);
        insert_matrix_diagonal_block(Q, Q_i, i + 1);
    }
    //std::cout<< "Q:"<<std::endl;
    //std::cout<< Q<<std::endl;
    /**
     * @brief 计算Mi,Mi的形式相似，只有T时间分配可能有区别，计算大M
     */
    Eigen::MatrixXd M = Eigen::MatrixXd::Zero(M_rows, M_cols);
    Eigen::MatrixXd M_inverse = Eigen::MatrixXd::Zero(M_rows, M_cols);
    Eigen::MatrixXd M_inverse_transpose = Eigen::MatrixXd::Zero(M_rows, M_cols);
    for (int i = 0; i < segments_num; i++)
    {
        T = T_allocation(i, 0);
        Eigen::MatrixXd Mi = Eigen::MatrixXd::Zero(Mi_rows, Mi_cols);
        Mi << 1, 0, 0, 0, 0, 0,
            0, 1, 0, 0, 0, 0,
            0, 0, 2, 0, 0, 0,
            1, T, pow(T, 2), pow(T, 3), pow(T, 4), pow(T, 5),
            0, 1, 2 * T, 3 * pow(T, 2), 4 * pow(T, 3), 5 * pow(T, 4),
            0, 0, 2, 6 * T, 12 * pow(T, 2), 20 * pow(T, 3);
        //M = set_matrix_diagonal_block(M, Mi, i + 1);
        insert_matrix_diagonal_block(M, Mi, i + 1);
    }
    //std::cout<< "M:"<<std::endl;
    //std::cout<< M<<std::endl;
    // 计算
    M_inverse = M.inverse();
    M_inverse_transpose = M_inverse.transpose();
    /**
     * @brief 计算C_transpose
     */
    Eigen::MatrixXd C_transpose = compute_C_transpose(segments_num);
    Eigen::MatrixXd C = C_transpose.transpose();
    /**
     * @brief 计算R = C*A_inverse_transpose*Q*A_inverse*C_transpose
     *    R: 8 * segments_num - 4 * (n - 1) X 8 * segments_num - 4 * (n - 1)
     */
    Eigen::MatrixXd R = C * M_inverse_transpose * Q * M_inverse * C_transpose;

    /**
     * @brief 获取 R_FF  R_FP  R_PF  R_PP
     *            R_FP_transpose
     *            R_PP_inverse
     */
    Eigen::MatrixXd R_FF = R.topLeftCorner(df_rows, df_rows);
    Eigen::MatrixXd R_FP = R.topRightCorner(df_rows, dp_rows);
    Eigen::MatrixXd R_PF = R.bottomLeftCorner(dp_rows, df_rows);
    Eigen::MatrixXd R_PP = R.bottomRightCorner(dp_rows, dp_rows);
    Eigen::MatrixXd R_FP_transpose = R_FP.transpose();
    Eigen::MatrixXd R_PP_inverse = R_PP.inverse();

    /**
     * @brief 计算d df， dp
     * //计算x y z的minimumsnap 轨迹，主要区别在于 d df dp值的不同，形式是一样的了（因为轨迹的段数已经定下来了）
     */

    // --------------------------------------------------------------------waypoints_x
    Eigen::MatrixXd dfp_x = Eigen::MatrixXd::Zero(dfp_rows, dfp_cols);
    Eigen::MatrixXd df_x = Eigen::MatrixXd::Zero(df_rows, dfp_cols);
    Eigen::MatrixXd dp_x = Eigen::MatrixXd::Zero(dp_rows, dfp_cols);

    df_x(0, 0) = waypoints_xyz(0, 0); // 起始点位置约束（已知）
    df_x(1, 0) = 0;                   // 起始点acc约束（已知）
    df_x(2, 0) = 0;                   // 起始点jerk约束（已知）

    for (int i = 3; i < (df_rows - 3); i++)
    {
        df_x(i, 0) = waypoints_xyz(i - 3 + 1, 0);
    }

    df_x(df_rows - 3, 0) = waypoints_xyz(waypoints_num - 1, 0); // 末端点位置约束（已知）
    df_x(df_rows - 2, 0) = 0;                                   // 末端点acc约束（已知）
    df_x(df_rows - 1, 0) = 0;                                   // 末端点jerk约束（已知）

    // -------------------------------------------------------------------waypoints_y
    Eigen::MatrixXd dfp_y = Eigen::MatrixXd::Zero(dfp_rows, dfp_cols);
    Eigen::MatrixXd df_y = Eigen::MatrixXd::Zero(df_rows, dfp_cols);
    Eigen::MatrixXd dp_y = Eigen::MatrixXd::Zero(dp_rows, dfp_cols);

    df_y(0, 0) = waypoints_xyz(0, 1); // 起始点位置约束（已知）
    df_y(1, 0) = 0;                   // 起始点acc约束（已知）
    df_y(2, 0) = 0;                   // 起始点jerk约束（已知）

    for (int i = 3; i < (df_rows - 3); i++)
    {
        df_y(i, 0) = waypoints_xyz(i - 3 + 1, 1);
    }

    df_y(df_rows - 3, 0) = waypoints_xyz(waypoints_num - 1, 1); // 末端点位置约束（已知）
    df_y(df_rows - 2, 0) = 0;                                   // 末端点acc约束（已知）
    df_y(df_rows - 1, 0) = 0;                                   // 末端点jerk约束（已知）

    // ------------------------------------------------------------------waypoints_z
    Eigen::MatrixXd dfp_z = Eigen::MatrixXd::Zero(dfp_rows, dfp_cols);
    Eigen::MatrixXd df_z = Eigen::MatrixXd::Zero(df_rows, dfp_cols);
    Eigen::MatrixXd dp_z = Eigen::MatrixXd::Zero(dp_rows, dfp_cols);

    df_z(0, 0) = waypoints_xyz(0, 2); // 起始点位置约束（已知）
    df_z(1, 0) = 0;                   // 起始点acc约束（已知）
    df_z(2, 0) = 0;                   // 起始点jerk约束（已知）

    for (int i = 3; i < (df_rows - 3); i++)
    {
        df_z(i, 0) = waypoints_xyz(i - 3 + 1, 2);
    }

    df_z(df_rows - 3, 0) = waypoints_xyz(waypoints_num - 1, 2); // 末端点位置约束（已知）
    df_z(df_rows - 2, 0) = 0;                                   // 末端点acc约束（已知）
    df_z(df_rows - 1, 0) = 0;                                   // 末端点jerk约束（已知）

    // ------------------------------------------------------------------------------x
    /**
     * @brief 求解dp dfp
     */
    dp_x = -R_PP_inverse * R_FP_transpose * df_x;
    for (int i = 0; i < df_rows; i++)
    {
        dfp_x(i, 0) = df_x(i, 0);
    }

    for (int i = 0; i < dp_rows; i++)
    {
        dfp_x(df_rows + i, 0) = dp_x(i, 0);
    }

    /**
     * @brief 求解d   d=M*a a=M_inverse*d
     *               d= C_transpose * dfp;
     */
    Eigen::MatrixXd d = C_transpose * dfp_x;

    /**
     * @brief 求解系数a
     */

    // result 赋值大小，防止下面对列操作出错
    result = Eigen::MatrixXd::Zero(M_inverse.rows(), 3);
    result.col(0) = (M_inverse * d).col(0);

    // ------------------------------------------------------------------------------y
    /**
     * @brief 求解dp_y dfp_y
     */
    dp_y = -R_PP_inverse * R_FP_transpose * df_y;
    for (int i = 0; i < df_rows; i++)
    {
        dfp_y(i, 0) = df_y(i, 0);
    }

    for (int i = 0; i < dp_rows; i++)
    {
        dfp_y(df_rows + i, 0) = dp_y(i, 0);
    }

    /**
     * @brief 求解d_y   d=M*a a=M_inverse*d
     *               d= C_transpose * dfp;
     */
    Eigen::MatrixXd d_y = C_transpose * dfp_y;

    /**
     * @brief 求解系数a_y
     */
    result.col(1) = (M_inverse * d_y).col(0);

    // ------------------------------------------------------------------------------z
    /**
     * @brief 求解dp_z dfp_z
     */
    dp_z = -R_PP_inverse * R_FP_transpose * df_z;
    for (int i = 0; i < df_rows; i++)
    {
        dfp_z(i, 0) = df_z(i, 0);
    }

    for (int i = 0; i < dp_rows; i++)
    {
        dfp_z(df_rows + i, 0) = dp_z(i, 0);
    }

    /**
     * @brief 求解d_z   d=M*a a=M_inverse*d
     *               d= C_transpose * dfp;
     */
    Eigen::MatrixXd d_z = C_transpose * dfp_z;

    /**
     * @brief 求解系数a_z
     */
    result.col(2) = (M_inverse * d_z).col(0);
}

/**
 * @brief publish waypoints
 ×        关于t的5次多项式
 * @param a 多项式系数
 ×        t 时间
 * @return none
 */
double MinimumJerk::polynomail_degree_5(Eigen::MatrixXd &a, double t)
{
    double sum = 0;
    double tt = 1;

    for (int i = 0; i <= 5; i++)
    {
        sum += tt * a(i, 0);
        tt *= t;
    }
    return sum;
}

/**
 * @brief get waypoints
 * @param void
 * @return none
 */
void MinimumJerk::get_waypoints(Eigen::MatrixXd &time_allocation)
{
    //std::cout<< "void MinimumSnap::get_waypoints(void):"<<std::endl ;

    nav_msgs::Path w;
    // 获取轨迹段数
    int segment_num = path_points_3d_.rows() - 1;
    traj_points_.header.frame_id = "/base_link";
    traj_points_.header.stamp = ros::Time::now();
    for (int i = 0; i < segment_num; i++)
    {
        double t = 0;
        // 一段一段轨迹
        Eigen::MatrixXd a = a_xyz_.block(6 * i, 0, 6, 1);
        Eigen::MatrixXd a_y = a_xyz_.block(6 * i, 1, 6, 1);
        Eigen::MatrixXd a_z = a_xyz_.block(6 * i, 2, 6, 1);
        //std::cout<< "aaaaaaaaaa:"<<a ;
        a_path_t = time_allocation(i, 0);
        for (double j = 0; j < a_path_t / 0.2 - 1; j++)
        {
            t += 0.2;
            geometry_msgs::PoseStamped pose_stamp;
            pose_stamp.header.frame_id = "/base_link";
            pose_stamp.header.stamp = ros::Time::now();
            pose_stamp.pose.position.x = polynomail_degree_5(a, t) + origin_x_;
            pose_stamp.pose.position.y = polynomail_degree_5(a_y, t) + origin_y_;
            pose_stamp.pose.position.z = polynomail_degree_5(a_z, t) + origin_z_;
            traj_points_.poses.push_back(pose_stamp);
        }
    }
}

/**
 * @brief publish waypoints
 * @param waypoints
 * @return none
 */
void MinimumJerk::publish_waypoints(void)
{
    //std::cout<< "void MinimumSnap::publish_waypoints(void):"<<std::endl;
    traj_points_array_.poses.clear();
    traj_points_array_.header.frame_id = "/base_link";
    traj_points_array_.header.stamp = ros::Time::now();
    geometry_msgs::Pose init_pose;
    init_pose.position.x = origin_x_;
    init_pose.position.y = origin_y_;
    init_pose.position.z = origin_z_;
    traj_points_array_.poses.push_back(init_pose);
    for (int i = 0; i < traj_points_.poses.size(); i++)
    {
        geometry_msgs::Pose p = traj_points_.poses[i].pose;
        double angle;

        if (i == (traj_points_.poses.size() - 1))
        {
            angle = atan2(traj_points_.poses[i].pose.position.y - traj_points_.poses[i - 1].pose.position.y, traj_points_.poses[i].pose.position.x - traj_points_.poses[i - 1].pose.position.x);
        }
        else
        {
            angle = atan2(traj_points_.poses[i + 1].pose.position.y - traj_points_.poses[i].pose.position.y, traj_points_.poses[i + 1].pose.position.x - traj_points_.poses[i].pose.position.x);
        }

        if (angle < 0)
        {
            angle += 2 * M_PI;
        }
        p.orientation = tf::createQuaternionMsgFromYaw(angle);
        traj_points_array_.poses.push_back(p);
    }
}

/**
 * @brief get waypoints
 * @param void
 * @return none
 */
nav_msgs::Path MinimumJerk::get_trajectory_points(void)
{
    return traj_points_;
}

/**
 * @brief get waypoints
 * @param void
 * @return none
 */
geometry_msgs::PoseArray MinimumJerk::get_trajectory_points_array(void)
{
    return traj_points_array_;
}

/**
 * @brief gset_origin_xyz
 * @param void
 * @return none
 */
void MinimumJerk::set_origin_xyz(double x, double y, double z)
{
    origin_x_ = x;
    origin_y_ = y;
    origin_z_ = z;
}
