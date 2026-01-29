#include "flash.h"
#include "pid.h"
#define FLASH_SECTION_INDEX       (127) //存储数据的扇区，倒数第一个
#define FLASH_PAGE_INDEX          (3)  //存储数据的页码，倒数第一个

extern PID_t AnglePID;
extern PID_t SpeedPID;



/**
	菜单参数保存
	使用思路:按下某个按键，然后改变参数，然后储存
*/
void menu_save(void)
{
    if(flash_check(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX))
    {
        flash_erase_page(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX);             
    }
    flash_buffer_clear(); 
    flash_union_buffer[0].float_type  = AnglePID.Ki;
    flash_union_buffer[1].float_type  = AnglePID.Kd;

    flash_union_buffer[2].float_type  = AnglePID.Kp;
		flash_union_buffer[3].float_type  = SpeedPID.Kp;
		flash_union_buffer[4].float_type  = SpeedPID.Ki;
		flash_union_buffer[5].float_type  = SpeedPID.Kd;

//    flash_union_buffer[6].float_type  = 
//    flash_union_buffer[7].float_type  = 
//    flash_union_buffer[8].float_type  = 
//    flash_union_buffer[9].float_type  = 

//    flash_union_buffer[10].float_type =
//    flash_union_buffer[11].float_type = 

//    flash_union_buffer[12].float_type = 

//    flash_union_buffer[13].float_type = 
//    flash_union_buffer[14].float_type = 

//    flash_union_buffer[15].float_type = 

//    flash_union_buffer[16].float_type = 
//    flash_union_buffer[17].float_type = 

//    flash_union_buffer[18].float_type = 
//    flash_union_buffer[19].float_type = 

    flash_write_page_from_buffer(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX);
}

/*
		将存储的数据取出
		使用思路：在初始化阶段使用，将存储的数据全部读出
*/
void menu_load(void)
{
    flash_read_page_to_buffer(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX); 
    AnglePID.Ki=flash_union_buffer[0].float_type;
    AnglePID.Kd=flash_union_buffer[1].float_type;
    AnglePID.Kp=flash_union_buffer[2].float_type;
    SpeedPID.Kp=flash_union_buffer[3].float_type;
    SpeedPID.Ki=flash_union_buffer[4].float_type;
    SpeedPID.Kd=flash_union_buffer[5].float_type;

//    =flash_union_buffer[6].float_type;
//    =flash_union_buffer[7].float_type;
//    =flash_union_buffer[8].float_type;
//    =flash_union_buffer[9].float_type;

//     = flash_union_buffer[10].float_type;
//    = flash_union_buffer[11].float_type;

//    = flash_union_buffer[12].float_type;

//     = flash_union_buffer[13].float_type;
//    = flash_union_buffer[14].float_type;

//     = flash_union_buffer[15].float_type;

//    = flash_union_buffer[16].float_type;
//     = flash_union_buffer[17].float_type;
//     = flash_union_buffer[18].float_type;
//     =  flash_union_buffer[19].float_type;
//    
    flash_buffer_clear();
}