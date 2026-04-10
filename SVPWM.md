



扇区判断方式：



## 1.  反正切法：  每个扇区相差六十度，根据角度直接算出扇区所在位置  计算量太大，太吃CPU，一般不用

      缺点：计算量大

      alpha：
	 目标电压矢量在 alpha-beta 平面中的相角，单位弧度。
	 公式：
	   alpha = atan2(v_beta, v_alpha)
	
	 atan2() 返回值位于 [-pi, +pi]
	 

`float alpha = atan2f(vBeta,vAlpha)`

	为了方便判断扇区，把 alpha 统一转换到 [0, 2*pi) 区间



`if(alpha < 0)`
`alpha += 6.28318530717956f;` //+2*pi
`if (alpha == 6.28318530717956f)`
`alpha = 0;`
	
	/* sector：
	 * 当前目标电压矢量所处的扇区编号。
	 * 在 SVM 中，alpha-beta 平面被分成 6 个 60 度扇区。
	 * 从0°开始 每六十度一个扇区 1-6
	 * 扇区判断公式可写成：
	 *   sector = floor(alpha / (pi/3)) + 1
	 *
	 * 代码里写成：
	 *   sector = floor(3*alpha / pi) + 1
	 */

`uint8_t sector = 3 * alpha / PI`
`sector++;`



## 2.  比较法：  Beta alpha 比较法  

       缺点：当`alpha`很小时，A会无限大导致报错

 `Beta`大于0，一定在123扇区； Beta小于0，一定在456扇区
 `alpha`大于0，一定在16扇区； alpha小于0，一定在34扇区、

 `Beta/alpha` 的值 `A`

 1：`0 < A < SQRT(3) `

 2：`SQRT(3) <  abs(A)`

 3：`-SQRT(3) < A < 0`

 4：`0 < A < SQRT(3)`

 5：`SQRT(3) <  abs(A)`

 6：`-SQRT(3) < A < 0`





 ##  3.  三条直线范围   最常用！！！！！

    1. 水平线  Beta = 0  即 alpha轴  A

    2. 与alpha轴成 60° 度夹角的直线 即 SQRT(3) * alpha - Beta = 0   B
        若该公式大于0，则位置直线右侧
    3. 与alpha轴成 120° 夹角的直线 即 -SQRT(3) * alpha - Beta = 0   C
        若该公式大于0，则位置直线左侧

    4.ABC 三者，直线大于0时结果为1 小于0时结果为0

    #define SQRT3 1.7320508f
    uint8_t sector;
    uint8_t N;
    float Ts = 2 * ARR; //控制周期
    /*
     * 期望作用顺序是 T0 - T1 - T2 - T0
     * 最小化开关损耗，每次只动一个开关的顺序
     */
    float T1, T2, T0; //从T0开始滞后，只有一个开关闭合的矢量作用时间 T1  另一个矢量作用时间 T2 
    float Ta, Tb, Tc; //A相作用时间 Ta  B相作用时间 Tb  C相作用时间 Tc


     //判断电压矢量与三条直线的位置关系

    uint8_t A = (vBeta > 0.0f) ? 1U : 0U;
    uint8_t B = ((SQRT3 * vAlpha - vBeta) > 0.0f) ? 1U : 0U;
    uint8_t C = ((-SQRT3 * vAlpha - vBeta) > 0.0f) ? 1U : 0U;
    
                                                                                         
     //计算N值：N = 4*C + 2*B + A

    N = 4*C + 2*B + A;

    float k = Ts / V_dc; 
    
    //根据N值确定扇区
  
    switch(N)
    {
        case 3:
        //扇区1
            sector = 1;
            T1 = k *(1.5f * vAlpha - 0.8660254f * vBeta)
            T2 = 1.7320508f * k * vBeta

            //过调制保护
            if(T1 + T2 > Ts){
                float scale = Ts / (T1 + T2);
                T1 *= scale;
                T2 *= scale;
            }

            T0 = Ts - T1 - T2；

            /*
             * 七段式计算各相导通时间
             * 扇区1：右边界V4(100) 左边界V6(110)
             * 顺序: 000 -> 100 -> 110 -> 111 -> 111 -> 110 -> 100 -> 000
             * 此顺序是为了保证一次只动一个开关
             * A相导通时间: T1 + T2 + T0 /2
             * B相导通时间: T2 + T0 /2
             * C相导通时间: T0/2
             */
            Ta = T1 + T2 + T0 /2;
            Tb = T2 + T0 /2;
            Tc = T0 /2；

            break;

        case 1:
        //扇区2
            sector = 2;
            T1 = k *(-1.5f * vAlpha + 0.8660254f * vBeta)
            T2 = k * (1.5f * vAlpha + 0.8660254f * vBeta)

            if(T1 + T2 > Ts){
                float scale = Ts / (T1 + T2);
                T1 *= scale;
                T2 *= scale;
            }

            T0 = Ts - T1 - T2；

            /*
             * 扇区2:右边界V6(110) 左边界V2(010)
             * 顺序: 000 -> 010 -> 110 -> 111 -> 111 -> 110 -> 010 -> 000
             * A相导通时间: T1  + T0 /2
             * B相导通时间: T1 + T2 + T0 /2;
             * C相导通时间: T0/2
             */
            Ta = T1 + T0 /2;
            Tb = T1 + T2 + T0 /2;
            Tc = T0 /2；

            break;

        case 5:
        //扇区3
            sector = 3;
            T1 = 1.7320508f * k * vBeta
            T2 = k *(-1.5f * vAlpha - 0.8660254f * vBeta)

            if(T1 + T2 > Ts){
                float scale = Ts / (T1 + T2);
                T1 *= scale;
                T2 *= scale;
            }

            T0 = Ts - T1 - T2；

            /*
             * 扇区3:右边界V2(010) 左边界V3(011)
             * 顺序: 000 -> 010 -> 011 -> 111 -> 111 -> 011 -> 010 -> 000
             * A相导通时间: T0 /2
             * B相导通时间: T1 + T2 + T0 /2;
             * C相导通时间: T2 + T0 /2；
             */
            Ta = T0 /2;
            Tb = T1 + T2 + T0 /2;
            Tc = T2 + T0 /2;

            break;

        case 4:
        //扇区4
            sector = 4;
            T1 =  k * ( -1.7320508f * vBeta)
            T2 = k *(-1.5f * vAlpha + 0.8660254f * vBeta)

            if(T1 + T2 > Ts){
                float scale = Ts / (T1 + T2);
                T1 *= scale;
                T2 *= scale;
            }

            T0 = Ts - T1 - T2；

             /*
             * 扇区4:右边界V3(011) 左边界V1(001)
             * 顺序: 000 -> 001 -> 011 -> 111 -> 111 -> 011 -> 001 -> 000
             * A相导通时间: T0 /2
             * B相导通时间: T1 + T0 /2
             * C相导通时间: T1 + T2 + T0 /2 
             */
            Ta = T0 /2;
            Tb = T1 + T0 /2;
            Tc = T1 + T2 + T0 /2；

            break;

        case 6:
        //扇区5
            sector = 5;
            T1 = k * (- 1.5f * vAlpha - 0.8660254f * vBeta)
            T2 = k *(1.5f * vAlpha - 0.8660254f * vBeta)

            if(T1 + T2 > Ts){
                float scale = Ts / (T1 + T2);
                T1 *= scale;
                T2 *= scale;
            }

            T0 = Ts - T1 - T2；

             /*
             * 扇区5:右边界V1(001) 左边界V5(101)
             * 顺序: 000 -> 001 -> 101 -> 111 -> 111 -> 101 -> 001 -> 000
             * A相导通时间: T2 + T0 /2
             * B相导通时间: T0 /2
             * C相导通时间: T1 + T2 + T0/2
             */
            Ta = T2 + T0 /2;
            Tb = T0 /2;
            Tc = T1 + T2 + T0/2；

            break;

        case 2:
        //扇区6
            sector = 6;
            T1 = k *(1.5f * vAlpha + 0.8660254f * vBeta)
            T2 = k *  ( -1.7320508f * vBeta)

            if(T1 + T2 > Ts){
                float scale = Ts / (T1 + T2);
                T1 *= scale;
                T2 *= scale;
            }

            T0 = Ts - T1 - T2；

            /*
             * 扇区6:右边界V5(101) 左边界V4(100)
             * 顺序: 000 -> 100 -> 101 -> 111 -> 111 -> 101 -> 100 -> 000
             * A相导通时间: T1 + T2 + T0 /2
             * B相导通时间: T0 /2
             * C相导通时间: T1 + T0 /2
             */
            Ta = T1 + T2 + T0 /2;
            Tb = T0 /2;
            Tc = T1 + T0 /2;

            break;

        default:
            sector = 0;  // 异常保护
            break;
    }

    //将导通时间转换为定时器比较值CCR

    if (sector > 0) {

         //非负保证
         if(Ta < 0) Ta = 0;
         if(Tb < 0) Tb = 0;
         if(Tc < 0) Tc = 0;

         /* 
          * 公式说明：
          * 在中心对齐模式下，计数器从 0 到 ARR 再从 ARR 到 0 循环
          * CCR的值是在计数器达到该值时输出高电平
          */
    
         CmpA = (uint16_t)(ARR - (Ta / 2.0f));
         CmpB = (uint16_t)(ARR - (Tb / 2.0f));
         CmpC = (uint16_t)(ARR - (Tc / 2.0f));

        
         //防溢出保证
         CmpA = (uint16_t)(tempA < 0 ? 0 : (tempA > ARR ? ARR : tempA));
         CmpB = (uint16_t)(tempB < 0 ? 0 : (tempB > ARR ? ARR : tempB));
         CmpC = (uint16_t)(tempC < 0 ? 0 : (tempC > ARR ? ARR : tempC));

    } else {
    // 零矢量处理 (例如全低电平或全高电平)
    // 这里设置为 50% 占空比作为默认安全状态
    CmpA = CmpB = CmpC = ARR / 2; 
    }
        // 写入硬件寄存器 
        // TIM1->CCR1 = CmpA; // A相
        // TIM1->CCR2 = CmpB; // B相
        // TIM1->CCR3 = CmpC; // C相








    





