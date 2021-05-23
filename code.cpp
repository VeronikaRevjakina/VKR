

#include <iostream>
#include <cmath>

#define PI 3.14159265  

using namespace std;

int main()
{
    //Расчитываем sigma
    int alpha = 1;
    int deltaq = 0;
    for (int i = 0; i <= k; i++){
        if (bearingn [i][1] != -1){ 
            deltaq += bearingn[i][1] - q;
			}
			}
    if (k_t == 0){
        k_t = 1;
		}
    
    float sigma_t = (alpha*deltaq)/k_t;
    
    //Расчитываем гамма^i_t 
    if (q ==0){
        q=0.01;
		}
    float gamma_t = 1/(q+sigma_t);
    
   
    //Считаем среднюю уверенность соседей q_sr
	/*
    int q_sum = 0;
    int k_i = k;
    int q_sr = q;
    for (int i = 0; i <= k; i++){
        if (bearingn [i][1] != 0){
            q_sum += bearingn [i][1];
		}
        else{
		k_i -= 1;
		}
	}
    if (k_i > 0){
	q_sr = q_sum/k_i;}
    else{
	q_sr = 1;} */
    
    
    //Считаем средневзвешенный курс соседей dbearingn_sr_w
    
    /*float dbearingn_sr_w = 0;
    float cos_sum_sr_w = 0;
    float sin_sum_sr_w = 0;
    float sin_sum_w = 0;
    float cos_sum_w = 0;
    float k_i_w = 0;
    for (int i = 0; i <= k; i++){
        if (q_sum != 0){
            q_i_sr_w = (bearingn[i][1])/q_sum;
		}
        else{
            q_i_sr_w = (bearingn[i][1]);
		}
        if (bearingn [i][1] > 0){
            //cos_sum_w += cos(math.radians(bearingn[i][0]))*q_i_sr_w;
            //sin_sum_w += sin(math.radians(bearingn[i][0]))*q_i_sr_w;
            k_i_w += 1;
		}
	}
    if (k_i_w != 0){
        cos_sum_sr_w = cos_sum_w*q_sum/k_i_w;
        sin_sum_sr_w = sin_sum_w*q_sum/k_i_w;
	}
	}*/
		
    
   // Рассчитываем сумму разностей
    float cos_delta_sum = 0;
    float sin_delta_sum = 0;
	float cos_bearingn =0;
	float sin_bearingn=0;
	

    float cos_bearing = cos(bearing* PI / 180);
    float sin_bearing = sin(bearing* PI / 180); 
    float cos_dbearing = cos(dbearing* PI / 180);
    float sin_dbearing = sin(dbearing* PI / 180);
    for (int i = 0; i <= k; i++){
        
        if (bearingn [i][1] != -1){
            cos_bearingn = cos(bearingn [i][0]* PI / 180);
            sin_bearingn = sin(bearingn [i][0]* PI / 180);
            cos_delta_sum = cos_delta_sum + cos_bearingn*bearingn [i][1] - cos_bearing*q;
            sin_delta_sum =sin_delta_sum + sin_bearingn*bearingn [i][1] - sin_bearing*q;
		}
	}
	
    //Рассчитываем курс в группе dbearingG исходя из данных группы
    //alpha - коэфициент, p - уверенность к курсу при пересчете от группы
    float dbearingG = 0;
	
    if (k_t == 0){
        dbearingG = dbearing;
	}
    else{ 
        float cos_db_G = cos_dbearing*(1-(sigma_t*gamma_t))+(alpha*gamma_t*cos_delta_sum)/k_t; 
        float sin_db_G = sin_dbearing*(1-(sigma_t*gamma_t))+(alpha*gamma_t*sin_delta_sum)/k_t;
        //cos_db_G = ((1-alpha)*cos_bearing*q + alpha*cos_sum_sr_w)/((1-alpha)*q + alpha*q_sr)
        //sin_db_G = ((1-alpha)*sin_bearing*q + alpha*sin_sum_sr_w)/((1-alpha)*q + alpha*q_sr)
        
        if (cos_db_G < -1){
		cos_db_G = -1;}
        if (cos_db_G > 1){
		cos_db_G = 1;}
        if (cos_db_G > 0 && sin_db_G > 0){
            dbearingG = (acos(cos_db_G))*180 / PI;
		}
        else if (cos_db_G > 0 && sin_db_G < 0){
            dbearingG = 360 - (acos(cos_db_G))*180 / PI;
		}
        else if (cos_db_G < 0 && sin_db_G > 0){
            dbearingG = 180 - (acos(cos_db_G))*180 / PI;
		}			
        else if (cos_db_G < 0 && sin_db_G < 0){
            dbearingG = 180 + (acos(cos_db_G))*180 / PI;
		}
	}
}
