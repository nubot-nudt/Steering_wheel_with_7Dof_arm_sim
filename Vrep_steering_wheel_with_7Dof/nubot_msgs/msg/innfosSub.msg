Header header
int32 firstNeedMode 
# second 为双臂时，第二个臂参数
int32 secondNeedMode
#needMode  1:v 2:p

#speed 和Pos 都是电机原始数据，考虑了减速比的
float64[8] firstSpeed   
float64[8] firstPos
#需要设置电机的位置
float64[8] secondSpeed   
float64[8] secondPos

int32 firstPosMode  
#是否是位置模式
int32 secondPosMode
