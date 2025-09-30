#ifndef _ADRC_HPP_
#define _ADRC_HPP_ 

namespace ADRC
{
    class Second_ADRC_
    {
        public:
            Second_ADRC_(float r, float h, float w0_, float wc_, float b0_) : R(r), H(h), wc(wc_), w0(w0_), b0(b0_){};

            float R;        //跟踪速度因子
            float V1;       //跟踪信号
            float V2;       //跟踪信号微分
            float H;        //离散系统的采样步长

            float kp;       //衍生参数 wc^2
            float kd;       //衍生参数  2*wc
            float beta1;      // 3*w0
            float beta2;      // 3*w0^2
            float beta3;      // w0^3
            float u0;
            float e;

            float z1;
            float z2;
            float z3;
            float u;

            float wc;
            float w0; 
            float b0;

            void TdFilter(float Input);
            float Second_LADRC(float tar, float ref);
        private:
            void Second_LESO(float feedback);
            void Second_LSEF();
    };
    class First_LADRC_ 
    {
        public:
            First_LADRC_(float h, float w0_, float wc_, float b0_, float z1_, float z2_, float u0_, float u_, float beta1_, float beta2_, float kp_) 
                : H(h), kp(kp_), beta1(beta1_), beta2(beta2_), u0(u0_), z1(z1_), z2(z2_), u(u_), wc(wc_), w0(w0_), b0(b0_){};
            
            float input;
            float feedback;
            
            float H;        //离散系统的采样步长
            float R;        //跟踪速度因子
            float V1;       //跟踪信号
            float V2;       //跟踪信号微分

            float kp;       //衍生参数 wc
            float beta1;      // 2*w0
            float beta2;      // w0^2
            float u0;
            float e1;
            float e2;
            float e3;
            float e4;

            float z1;
            float z2;
            float u;

            float wc;
            float w0; 
            float b0;

            float First_LADRC(float tar, float ref);

        private:
            void First_LESO(float feedback);
            void First_LSEF();
            void Limit_max(float *out, float max_out);  //限幅
    };
    
}

#endif
