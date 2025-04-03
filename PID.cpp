/*
 * PID.cpp
 *
 *  Created on: Feb 19, 2025
 *      Author: Sezakiaoi
 */

#include <PID.h>

/**
 * @brief PIDへのゲインの入力
 *
 * @param[in] gain_p Pゲイン
 * @param[in] gain_p Iゲイン
 * @param[in] gain_p Dゲイン
 *
 * @return None
 */
void PID::Setup(float gain_p, float gain_i, float gain_d){

	this->gain_p = gain_p;
	this->gain_i = gain_i;
	this->gain_d = gain_d;
}

/**
 * @brief PIDの計算
 *
 * @param[in] goal   目標値
 * @param[in] now    現在値
 * @param[in] time   実行間隔（微積分に使用）
 *
 * @return float 制御結果
 */
float PID::Calc(float goal, float now, float time){

	float error = 0.0;
	float diff = 0.0;


	error = goal - now;
	integral += (error + pre_error) / 2.0 * time;
	diff = (pre_error - error) / time;

	pre_error = error;

	return error * gain_p + integral * gain_i + diff * gain_d;
}

/**
 * @brief PIDの初期化
 *
 * @return None
 */
void PID::Reset(){

	gain_p = 0;
	gain_i = 0;
	gain_d = 0;
	integral  = 0;
	pre_error = 0;
}
