#include <vector>
#include <iostream>
#include <csignal>

namespace openvslam{
class MATRIX {
public:
    int r, c;
    std::vector<std::vector<double>> mat;
    double EOL = 1e-9;
    bool inv_possible = false;

    MATRIX(){}

    MATRIX(int r, int c, int d) {
        this->r = r;
        this->c = c;
        this->mat.resize(r, std::vector<double>(c, 0));
        if (d == 1) {
            for (int i = 0; i < std::min(r, c); ++i)
                this->mat[i][i] = 1;
        }
    }

    void getRowCol()
    {
        std::cout<<this->r<<","<<this->c<<std::endl;
    }

    void print() {
        for (int i = 0; i < r; ++i) {
            for (int j = 0; j < c; ++j) {
                std::cout << mat[i][j] << " \n"[j == c - 1];
            }
        }
    }

    MATRIX operator*(MATRIX b) {
        if (c != b.r) {
            std::cout << "can't multiply\n"<<b.r;
            exit(0);
        }

        int row = r;
        int col = b.c;


//        std::cout << r << " " << c << " " << b.r << " " << b.c << "\n";


        MATRIX c(row, col, 0);

//        std::cout << "hiiiii \n";


        for (int i = 0; i < row; ++i) {

            for (int j = 0; j < col; ++j) {

                for (int k = 0; k < b.r; ++k) {

//                    std::cout << i  << " " << j << " " << k << " " << "\n";

                    c.mat[i][j] = c.mat[i][j] + mat[i][k] * b.mat[k][j];
                    if(c.mat[i][j] == NAN || c.mat[i][j] == INFINITY || c.mat[i][j] == -(INFINITY)){
                        throw raise(0);

                    }


                }
            }
        }



        return c;
    }
    MATRIX operator*(double b) {
        int row = this-> r;
        int col = this-> c;
        MATRIX c(row, col, 0);

        for (int i = 0; i < row; ++i) {
            for (int j = 0; j < col; ++j)
                    c.mat[i][j] = this -> mat[i][j] * b;
            }


        return c;
    }

    MATRIX operator+(MATRIX b) {
        if (c != b.c or r != b.r) {
            std::cout << "can't sum\n";
            exit(0);
        }

        MATRIX C(r, c, 0);

        for (int i = 0; i < r; ++i) {
            for (int j = 0; j < c; ++j) {

                C.mat[i][j] = mat[i][j] + b.mat[i][j];

            }
        }

        return C;
    }

    MATRIX operator-(MATRIX b) {
        if (c != b.c or r != b.r) {
            std::cout << "can't sum\n";
            exit(0);
        }

        MATRIX C(r, c, 0);

        for (int i = 0; i < r; ++i) {
            for (int j = 0; j < c; ++j) {

                C.mat[i][j] = mat[i][j] - b.mat[i][j];

            }
        }

        return C;
    }


    MATRIX inverse(){

        if(r != 3 or c != 3){
            std::cout << "cant possible with this function";
            return *this;
        }

        double determinant = 0;

        for(int i = 0; i < 3; i++)
            determinant = determinant + (mat[0][i] * (mat[1][(i+1)%3] * mat[2][(i+2)%3] - mat[1][(i+2)%3] * mat[2][(i+1)%3]));

        MATRIX m(3, 3, 0);
        for(int i = 0; i < 3; i++){
            for(int j = 0; j < 3; j++){
                m.mat[i][j] = ((mat[(j+1)%3][(i+1)%3] * mat[(j+2)%3][(i+2)%3]) - (mat[(j+1)%3][(i+2)%3] * mat[(j+2)%3][(i+1)%3]))/ determinant;

            }
        }
        return m    ;
    }
//
//    static void getCofactor(MATRIX &A, MATRIX &temp, int p, int q, int n) {
//        int i = 0, j = 0;
//
//        // Looping for each element of the matrix
//        for (int row = 0; row < n; row++) {
//            for (int col = 0; col < n; col++) {
//                //  Copying into temporary matrix only those element
//                //  which are not in given row and column
//                if (row != p && col != q) {
//                    temp.mat[i][j++] = A.mat[row][col];
//
//                    // Row is filled, so increase row index and
//                    // reset col index
//                    if (j == n - 1) {
//                        j = 0;
//                        i++;
//                    }
//                }
//            }
//        }
//    }
//
//    double determinant(MATRIX &A, int n) {
//        double D = 0; // Initialize result
//
//        //  Base case : if matrix contains single element
//        if (n == 1)
//            return A.mat[0][0];
//
//        MATRIX temp(r, r, 0); // To store cofactors
//
//        int sign = 1;  // To store sign multiplier
//
//        // Iterate for each element of first row
//        for (int f = 0; f < n; f++) {
//            // Getting Cofactor of A[0][f]
//            getCofactor(A, temp, 0, f, n);
//            D += sign * A.mat[0][f] * determinant(temp, n - 1);
//
//            // terms are to be added with alternate sign
//            sign = -sign;
//        }
//
//        return D;
//    }
//
//    void adjoint(MATRIX &A, MATRIX &adj) {
//        int N = A.r;
//
//        if (N == 1) {
//            adj.mat[0][0] = 1;
//            return;
//        }
//
//        // temp is used to store cofactors of A[][]
//        int sign = 1;
//        MATRIX temp(N, N, 0);
//
//        for (int i = 0; i < N; i++) {
//            for (int j = 0; j < N; j++) {
//                // Get cofactor of A[i][j]
//                getCofactor(A, temp, i, j, N);
//
//                // sign of adj[j][i] positive if sum of row
//                // and column indexes is even.
//                sign = ((i + j) % 2 == 0) ? 1 : -1;
//
//                // Interchanging rows and columns to get the
//                // transpose of the cofactor matrix
//                adj.mat[j][i] = (sign) * (determinant(temp, N - 1));
//            }
//        }
//    }

    MATRIX operator^(int x) {

        MATRIX C(c, r, 0);

        for (int i = 0; i < r; ++i) {
            for (int j = 0; j < c; ++j) {
                C.mat[j][i] = mat[i][j];
            }
        }

        return C;
    }
};
}


