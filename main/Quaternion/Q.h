/*
 Author : Yuta Takayasu
 Date : 2022/8/5

    Q library.
    
*/


#pragma once

class Q
{
    private:
        double dt;
    
    public:
        double v[4];
        double dpsi;
        void init();
        void update(double *acc,int real_dt);
        void norm();
        void calc_Euler(double *euler);
        void vecRotate(double *vec, double *result);

    Q(){
        v[0] = 1.0;v[1] = 0;v[2] = 0;v[3] = 0;
    }
};

void IRAM_ATTR Q::init(){
    dt = 0.001;
    v[0] = 1.0;v[1] = 0;v[2] = 0;v[3] = 0;
}

void IRAM_ATTR Q::update(double *acc,int real_dt){
    double dt = (double)real_dt / 1000000.0;


    double rotateArray[4][4] = { {0,        -acc[0],    -acc[1],    -acc[2] },
                                {acc[0],   0,          acc[2],     -acc[1] },
                                {acc[1],   -acc[2],    0,          acc[0]  },
                                {acc[2],   acc[1],     -acc[0],    0       }};


    double qnew[4];
    for(int i=0;i < 4;i++)qnew[i] = 0;

    for(int i = 0;i < 4;i++){
        qnew[i] = v[i];
        for(int k = 0;k < 4;k++){
            qnew[i] += (v[k]*rotateArray[i][k])/2.0f*dt;
        }
    }

    for(int i=0;i < 4;i++)v[i] = qnew[i];

    norm();

    return;
}

void IRAM_ATTR Q::norm(){
    double sca = v[0]*v[0] + v[1]*v[1] + v[2]*v[2] + v[3]*v[3];
    sca = sqrt(sca);
    v[0] /= sca; v[1] /= sca; v[2] /= sca; v[3] /= sca;
    return;
}

void IRAM_ATTR Q::calc_Euler(double *euler){
	euler[0] = asin(2*(v[0]*v[1]-v[2]*v[3]));
	euler[1] = atan2(2*(v[1]*v[3]+v[0]*v[2]),v[0]*v[0]-v[1]*v[1]-v[2]*v[2]+v[3]*v[3]);
	euler[2] = atan2(2*(v[1]*v[2]+v[0]*v[3]),v[0]*v[0]-v[1]*v[1]+v[2]*v[2]-v[3]*v[3]);
}

void IRAM_ATTR Q::vecRotate(double *vec, double *result){
    double veq[4];
    double tmp[4];
    double inv[4];
    double req[4];

    inv[0] = v[0]; inv[1] = -v[1]; inv[2] = -v[2]; inv[3] = -v[3];
    veq[0] = 0; veq[1] = vec[0]; veq[2] = vec[1]; veq[3] = vec[2];
    
    tmp[0] = v[0] * veq[0] - v[1] * veq[1] - v[2] * veq[2] - v[3] * veq[3];
    tmp[1] = v[1] * veq[0] + v[0] * veq[1] - v[3] * veq[2] + v[2] * veq[3];
    tmp[2] = v[2] * veq[0] + v[3] * veq[1] + v[0] * veq[2] - v[1] * veq[3];
    tmp[3] = v[3] * veq[0] - v[2] * veq[1] + v[1] * veq[2] + v[0] * veq[3];

    req[0] = tmp[0] * inv[0] - tmp[1] * inv[1] - tmp[2] * inv[2] - tmp[3] * inv[3];
    req[1] = tmp[1] * inv[0] + tmp[0] * inv[1] - tmp[3] * inv[2] + tmp[2] * inv[3];
    req[2] = tmp[2] * inv[0] + tmp[3] * inv[1] + tmp[0] * inv[2] - tmp[1] * inv[3];
    req[3] = tmp[3] * inv[0] - tmp[2] * inv[1] + tmp[1] * inv[2] + tmp[0] * inv[3];
    
    result[0] = req[1]; result[1] = req[2]; result[2] = req[3];

    return;
}