#include <iostream>
#include <vector>

#define GNUPLOT_IMPLEMENTATION
#include "gnuplot.hpp"

#define LINA_IMPLEMENTATION
#include "lina.hpp"

using lina::Matrixf;

#define STATE_DIM   2
#define MPC_N       2


float run_mpc(const Matrixf& x, const Matrixf& M, const Matrixf& C,
    const Matrixf& F, const Matrixf& R_bar, const Matrixf& Q_bar)
{
    const auto G{M.t() * Q_bar * M};
    const auto H{R_bar + C.t() * Q_bar * C};
    const auto E{C.t() * Q_bar * M};

    std::cout << "H: " << H << "\n";
    std::cout << "H.inv: " << H.inv() << "\n";
    return (H.inv() * E * x * -1.0f).at(0);
}

int main()
{
    Matrixf A{STATE_DIM, STATE_DIM};
    Matrixf B{STATE_DIM, 1};
    Matrixf F{STATE_DIM, STATE_DIM};
    Matrixf Q{STATE_DIM, STATE_DIM};
    Matrixf R{1, 1};
    Matrixf Q_bar{STATE_DIM * (MPC_N + 1), STATE_DIM * (MPC_N + 1)};
    Matrixf R_bar{MPC_N,  MPC_N};
    Matrixf M{STATE_DIM * (MPC_N + 1), STATE_DIM};
    Matrixf C{STATE_DIM * (MPC_N + 1), MPC_N};

    Matrixf init_x{STATE_DIM, 1};
    Matrixf x{STATE_DIM, 1};

    A << 1.f, 0.5f,
         0.f, 2.f;
    B << 0.f, 0.5f;
    F << 5.0f, 0.0f,
         0.0f, 5.0f;
    Q << 0.1f, 0.0f,
         0.0f, 0.1f;
    R << 0.1f;
    init_x << 5.f, 5.f;

    Matrixf As[MPC_N + 1];
    As[0] = Matrixf::eye(STATE_DIM, STATE_DIM);
    for (int i = 0; i <= MPC_N; ++i) {
        // compute M
        if (i >= 1) As[i] = As[i - 1] * A;
        M.block(i * STATE_DIM, 0, STATE_DIM, STATE_DIM) = As[i];
        // compute C
        if (i > 0) {
            for (int j = 0; j < i; ++j) {
                C.block(i * STATE_DIM, j, STATE_DIM, 1) = As[i - j - 1] * B;
            }
        }

        if (i < MPC_N) {
            R_bar.block(i, i, 1, 1) = R;
            Q_bar.block(i * STATE_DIM, i * STATE_DIM, STATE_DIM, STATE_DIM) = Q;
        } else {
            Q_bar.block(i * STATE_DIM, i * STATE_DIM, STATE_DIM, STATE_DIM) = F;
        }
    }

    std::cout << "A: " << A << "\n";
    std::cout << "B: " << B << "\n";
    std::cout << "M: " << M << "\n";
    std::cout << "C: " << C << "\n";
    std::cout << "Q_bar: " << Q_bar << "\n";
    std::cout << "R_bar: " << R_bar << "\n";
    std::cout << "init_x: " << init_x << "\n";

    x = init_x;
    std::vector<float> data;
    for (int i = 0; i < 10; ++i) {
        const auto control{run_mpc(x, M, C, F, R_bar, Q_bar)};
        data.push_back(x.at(0));
        x = A * x + B * control;
    }

    gp::Plotter p;
    p.line(data);
    return 0;
}
