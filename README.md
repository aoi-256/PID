# PID
PID制御用の簡単なコードです

## 使い方

実行時間が固定の場合は定数として、可変の場合は毎回取得して渡すと精度が安定すると思います

```cpp
#include "PID.h"

PID pid;

float now_value = 0.0;
float goal_value = 100.0;
float time = 0.1;

int main(void){

	pid.Setup(0.1, 0.001, 0.1);

  while(1){

  	now_value += pid.Calc(goal_value, now_value, time);

    //値の出力や送信をする
  }
}
```

