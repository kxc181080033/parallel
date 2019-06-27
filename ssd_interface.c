/* 
* Contributors: Youngjae Kim (youkim@cse.psu.edu)
*               Aayush Gupta (axg354@cse.psu.edu_
*   
* In case if you have any doubts or questions, kindly write to: youkim@cse.psu.edu 
*
* This source plays a role as bridiging disksim and flash simulator. 
* 
* Request processing flow: 
*
*  1. Request is sent to the simple flash device module. 
*  2. This interface determines FTL type. Then, it sends the request
*     to the lower layer according to the FTL type. 
*  3. It returns total time taken for processing the request in the flash. 
*
*/

#include "ssd_interface.h"
#include "disksim_global.h"
#include "dftl.h"

extern int merge_switch_num;
extern int merge_partial_num;
extern int merge_full_num;
int old_merge_switch_num = 0;
int old_merge_partial_num = 0;
int old_merge_full_num= 0;
int old_flash_gc_read_num = 0;
int old_flash_erase_num = 0;
int req_count_num = 1;
int cache_hit, rqst_cnt;
int flag1 = 1;
int count = 0;

int page_num_for_2nd_map_table;

//#define MAP_REAL_MAX_ENTRIES 6552// real map table size in bytes
//#define MAP_GHOST_MAX_ENTRIES 1640// ghost_num is no of entries chk if this is ok

#define CACHE_MAX_ENTRIES 300
//int ghost_arr[MAP_GHOST_MAX_ENTRIES];
//int real_arr[MAP_REAL_MAX_ENTRIES];
int cache_arr[CACHE_MAX_ENTRIES];


/***********************************************************************
 Variables for statistics    
***********************************************************************/
unsigned int cnt_read = 0;
unsigned int cnt_write = 0;
unsigned int cnt_delete = 0;
unsigned int cnt_evict_from_flash = 0;
unsigned int cnt_evict_into_disk = 0;
unsigned int cnt_fetch_miss_from_disk = 0;
unsigned int cnt_fetch_miss_into_flash = 0;

double sum_of_queue_time = 0.0;
double sum_of_service_time = 0.0;
double sum_of_response_time = 0.0;
unsigned int total_num_of_req = 0;



/***********************************************************************
 Mapping table
***********************************************************************/
int real_min = -1;
int real_max = 0;

/***********************************************************************
 Cache
***********************************************************************/
int cache_min = -1;
int cache_max = 0;

/***********************************************************************
 *    author: ymb    部分常用变量
 ***********************************************************************/
int translation_read_num = 0;
int translation_write_num = 0;
int MAP_REAL_MAX_ENTRIES= 0 ;
int MAP_GHOST_MAX_ENTRIES= 0 ;
int MAP_SEQ_MAX_ENTRIES= 0 ; 
int MAP_SECOND_MAX_ENTRIES= 0 ; 
int *real_arr = NULL;
int *ghost_arr = NULL;
int operation_time = 0;

int request_cnt = 0;
int cache_cmt_hit = 0;
int cache_scmt_hit = 0;
int cache_slcmt_hit = 0;
int maxentry = 0;

#define PRE_THRESHOLD 2         //预取判断阈值
#define NUM_ENTRIES_PER_TIME 8  //预取窗大小

/***********************************************************************
 *    author: zj    封装 callFsim的代码函数
 ***********************************************************************/
void SecnoToPageno(int secno,int scount,int *blkno,int *bcount);

/***********************************************************************
 *    author: zj  封装 DFTL的代码函数
 ***********************************************************************/
void DFTL_Hit_Real_CMT(int blkno);
void DFTL_Hit_Ghost_CMT(int blkno);
void DFTL_init_arr();
void DFTL_Real_Cmt_Full();
void DFTL_Ghost_Cmt_Full();


/***********************************************************************
 *    author: ymb
 *  封装 WRTL的代码函数
 ***********************************************************************/
int init_flag=0;
int WRFTL_Window_Size=0; //WRFTL优先置换区大小
double WRFTL_Tau=0.3; //通过Tau调控window_size比例
// int warm_flag; ADFTL定义过
void WRFTL_Scheme(int *pageno,int *req_size,int operation);
void WRFTL_init_arr();
void WRFTL_Hit_WCMT(int blkno, int operation);
void WRFTL_Move_RCMT2MRU(int blkno, int operation);
void WRFTL_Move_RCMT2WCMT(int blkno, int operation);
void WRFTL_Write_Pre_Load(int *pageno, int *req_size);
void WRFTL_Read_Pre_Load(int *pageno, int *req_size);
void WRFTL_WCMT_Is_Full();
void WRFTL_Remove_Entry_In_WCMT();
void WRFTL_RCMT_Is_Full();
int WRFTL_Find_Victim_In_WCMT();
int not_in_cache(unsigned int pageno);

double hot_ratio = 0.1;
Node *WRFTL_Head=NULL;



// Interface between disksim & fsim 

void reset_flash_stat()
{
    flash_read_num = 0;
    flash_write_num = 0;
    flash_gc_read_num = 0;
    flash_gc_write_num = 0; 
    flash_erase_num = 0;
    flash_oob_read_num = 0;
    flash_oob_write_num = 0; 
}

FILE *fp_flash_stat;
FILE *fp_gc;
FILE *fp_gc_timeseries;
double gc_di =0 ,gc_ti=0;


double calculate_delay_flash()
{
    double delay;
    double read_delay, write_delay;
    double erase_delay;
    double gc_read_delay, gc_write_delay;
    double oob_write_delay, oob_read_delay;

    oob_read_delay  = (double)OOB_READ_DELAY  * flash_oob_read_num;
    oob_write_delay = (double)OOB_WRITE_DELAY * flash_oob_write_num;

    read_delay     = (double)READ_DELAY  * flash_read_num; 
    write_delay    = (double)WRITE_DELAY * flash_write_num; 
    erase_delay    = (double)ERASE_DELAY * flash_erase_num; 

    gc_read_delay  = (double)GC_READ_DELAY  * flash_gc_read_num; 
    gc_write_delay = (double)GC_WRITE_DELAY * flash_gc_write_num; 


    delay = read_delay + write_delay + erase_delay + gc_read_delay + gc_write_delay + 
        oob_read_delay + oob_write_delay;

    if( flash_gc_read_num > 0 || flash_gc_write_num > 0 || flash_erase_num > 0 ) {
        gc_ti += delay;
    }
    else {
        gc_di += delay;
    }

    if(warm_done == 1){
        fprintf(fp_gc_timeseries, "%d\t%d\t%d\t%d\t%d\t%d\n", 
        req_count_num, merge_switch_num - old_merge_switch_num, 
        merge_partial_num - old_merge_partial_num, 
        merge_full_num - old_merge_full_num, 
        flash_gc_read_num,
        flash_erase_num);

        old_merge_switch_num = merge_switch_num;
        old_merge_partial_num = merge_partial_num;
        old_merge_full_num = merge_full_num;
        req_count_num++;
    }

    reset_flash_stat();

    return delay;
}


/***********************************************************************
 Initialize Flash Drive 
***********************************************************************/

void initFlash()
{
    blk_t total_blk_num;
    blk_t total_util_blk_num;
    blk_t total_extr_blk_num;

    // total number of sectors    
    total_util_sect_num  = flash_numblocks;
    total_extra_sect_num = flash_extrblocks;
    total_sect_num = total_util_sect_num + total_extra_sect_num; 

    // total number of blocks 
    total_blk_num = total_sect_num / SECT_NUM_PER_BLK;     // total block number
    total_util_blk_num = total_util_sect_num / SECT_NUM_PER_BLK;    // total unique block number

    global_total_blk_num = total_util_blk_num;

    total_extr_blk_num = total_blk_num - total_util_blk_num;        // total extra block number

    ASSERT(total_extr_blk_num != 0);

    if (nand_init(total_blk_num, 3) < 0) {
        EXIT(-4); 
    }

    switch(ftl_type){

        // pagemap
        case 1: ftl_op = pm_setup(); break;
        // blockmap
        //case 2: ftl_op = bm_setup(); break;
        // o-pagemap 
        case 3: ftl_op = opm_setup(); break;
        // fast
        case 4: ftl_op = lm_setup(); break;

        default: break;
    }

    ftl_op->init(total_util_blk_num, total_extr_blk_num);

    nand_stat_reset();
}

void printWearout()
{
    int i;
    FILE *fp = fopen("wearout", "w");
    
    for(i = 0; i<nand_blk_num; i++)
    {
        fprintf(fp, "%d %d\n", i, nand_blk[i].state.ec); 
    }

    fclose(fp);
}


void endFlash()
{
    nand_stat_print(outputfile);
    ftl_op->end;
    nand_end();
}  

/***********************************************************************
 Send request (lsn, sector_cnt, operation flag)
***********************************************************************/

void send_flash_request(int start_blk_no, int block_cnt, int operation, int mapdir_flag)
{
    int size;
    //size_t (*op_func)(sect_t lsn, size_t size);
    size_t (*op_func)(sect_t lsn, size_t size, int mapdir_flag);

    if((start_blk_no + block_cnt) >= total_util_sect_num){
        printf("start_blk_no: %d, block_cnt: %d, total_util_sect_num: %d\n", start_blk_no, block_cnt, total_util_sect_num);
        exit(0);
    }

    switch(operation){

    //write
    case 0:

        op_func = ftl_op->write;
        while (block_cnt> 0) {
            size = op_func(start_blk_no, block_cnt, mapdir_flag);
            start_blk_no += size;
            block_cnt-=size;
        }
        break;
    //read
    case 1:

        op_func = ftl_op->read;
        while (block_cnt> 0) {
            size = op_func(start_blk_no, block_cnt, mapdir_flag);
            start_blk_no += size;
            block_cnt-=size;
        }
        break;

        default: 
        break;
    }
}

void find_real_max()
{
    int i; 

    for(i=0;i < MAP_REAL_MAX_ENTRIES; i++) {
        if(opagemap[real_arr[i]].map_age > opagemap[real_max].map_age) {
            real_max = real_arr[i];
        }
    }
}

void find_real_min()
{
    int i,index; 
    int temp = 99999999;

    for(i=0; i < MAP_REAL_MAX_ENTRIES; i++) {
            if(opagemap[real_arr[i]].map_age <= temp) {
                real_min = real_arr[i];
                temp = opagemap[real_arr[i]].map_age;
                index = i;
            }
    }    
}

int find_min_ghost_entry()
{
    int i; 

    int ghost_min = 0;
    int temp = 99999999; 

    for(i=0; i < MAP_GHOST_MAX_ENTRIES; i++) {
        if(ghost_arr[i]>0)
        {
            if( opagemap[ghost_arr[i]].map_age <= temp) {
            ghost_min = ghost_arr[i];
            temp = opagemap[ghost_arr[i]].map_age;
            }
        }
    }
    // for(i=0; i < MAP_GHOST_MAX_ENTRIES; i++) {
    //     if( opagemap[ghost_arr[i]].map_age <= temp) {
    //     ghost_min = ghost_arr[i];
    //     temp = opagemap[ghost_arr[i]].map_age;
    //     }
    // }
    return ghost_min;
}

void init_arr()
{

    int i;
    for( i = 0; i < MAP_REAL_MAX_ENTRIES; i++) {
        real_arr[i] = -1;
    }
    for( i = 0; i < MAP_GHOST_MAX_ENTRIES; i++) {
        ghost_arr[i] = -1;
    }
    for( i = 0; i < CACHE_MAX_ENTRIES; i++) {
        cache_arr[i] = -1;
    }

}

int search_table(int *arr, int size, int val) 
{
    int i;
    for(i =0 ; i < size; i++) {
        if(arr[i] == val) {
            return i;
        }
    }

    printf("shouldnt come here for search_table()=%d,%d",val,size);
    for( i = 0; i < size; i++) {
    if(arr[i] != -1) {
        printf("arr[%d]=%d ",i,arr[i]);
    }
    }
    exit(1);
    return -1;
}

int find_free_pos( int *arr, int size)
{
    int i;
    for(i = 0 ; i < size; i++) {
        if(arr[i] == -1) {
            return i;
        }
    } 
    printf("shouldnt come here for find_free_pos()");
    exit(1);
    return -1;
}

void find_min_cache()
{
    int i; 
    int temp = 999999;

    for(i=0; i < CACHE_MAX_ENTRIES ;i++) {
        if(opagemap[cache_arr[i]].cache_age <= temp ) {
            cache_min = cache_arr[i];
            temp = opagemap[cache_arr[i]].cache_age;
        }
    }
}

int youkim_flag1=0;

double callFsim(unsigned int secno, int scount, int operation)
{
    double delay; 
    int bcount;
    unsigned int blkno; // pageno for page based FTL
    int cnt,z; int min_ghost;


    if(ftl_type == 1){ }

    if(ftl_type == 3) {
        page_num_for_2nd_map_table = (opagemap_num / MAP_ENTRIES_PER_PAGE);
        
        // if(youkim_flag1 == 0 ) {
        //     youkim_flag1 = 1;
        //     init_arr();
        // }

        if((opagemap_num % MAP_ENTRIES_PER_PAGE) != 0){
            page_num_for_2nd_map_table++;
        }
    }
    //扇区和页对齐,转化 
    SecnoToPageno(secno,scount,&blkno,&bcount);

    cnt = bcount;

    switch(operation)
    {
        //write/read
        case 0:
        case 1:

        while(cnt > 0)
        {
            cnt--;

            switch(ftl_type)
            {
                case 1:
                    // page based FTL
                    send_flash_request(blkno*4, 4, operation, 1); 
                    blkno++;
                    break;
                case 2:
                    // blck based FTL
                    send_flash_request(blkno*4, 4, operation, 1); 
                    blkno++;
                    break;
                case 4: 
                    // FAST scheme 
                    if(operation == 0){ write_count++; }
                    else {read_count++;}
                    send_flash_request(blkno*4, 4, operation, 1); //cache_min is a page for page baseed FTL
                    blkno++;
                    break;
                case 3:
                    // SDFTL scheme
                    // SDFTL_Scheme(&blkno,&cnt,operation);
                    // DFTL scheme
                    //DFTL_Scheme(&blkno,&cnt,operation);
                    // CPFTL scheme
                    // CPFTL_Scheme(&blkno,&cnt,operation);
                    // ADFTL scheme
                    // ADFTL_Scheme(&blkno,&cnt,operation);
                    // WRFTL scheme
                    WRFTL_Scheme(&blkno,&cnt,operation);
                    // PPFTLscheme   用于对比的，纯页级映射，切完全加载如RAM的FTL
                    //PPFTL_Scheme(&blkno,&cnt,operation);
                    break;
            }
        }
        break;
    }

    delay = calculate_delay_flash();
    return delay;
}

/************************CallFsim预处理函数**************************/
void SecnoToPageno(int secno,int scount,int *blkno,int *bcount)
{
    switch(ftl_type){
        case 1:
            // page based FTL 
            *blkno = secno / 4;
            *bcount = (secno + scount -1)/4 - (secno)/4 + 1;
            break;
        case 2:
            // block based FTL 
            *blkno = secno/4;
            *bcount = (secno + scount -1)/4 - (secno)/4 + 1;      
            break;
        case 3:
            // o-pagemap scheme
            *blkno = secno / 4;
            *blkno += page_num_for_2nd_map_table;
            *bcount = (secno + scount -1)/4 - (secno)/4 + 1;
            break;
        case 4:
            // FAST scheme
            *blkno = secno/4;
            *bcount = (secno + scount -1)/4 - (secno)/4 + 1;
            break;		
    }
}



/***********************************************************************
 *                    DFTL  主函数逻辑实现
 ***********************************************************************/ 
void DFTL_Scheme(int *pageno,int *req_size,int operation)
{
    int blkno=(*pageno),cnt=(*req_size);
    int pos=-1,pos_real=-1,pos_ghost=-1;
    int min_ghost=-1;

    //变量的初始化
    if (init_flag==0)
    {
        MAP_REAL_MAX_ENTRIES = 6552;  // real map table size in bytes
        MAP_GHOST_MAX_ENTRIES = 1640;
        real_arr=(int *)malloc(sizeof(int)*MAP_REAL_MAX_ENTRIES);
        ghost_arr=(int *)malloc(sizeof(int)*MAP_GHOST_MAX_ENTRIES);
        DFTL_init_arr();
        //request_cnt = rqst_cnt;
        init_flag=1;
    }
    //1. pagemap in SRAM 
    rqst_cnt++;
    if((opagemap[blkno].map_status == MAP_REAL) || (opagemap[blkno].map_status == MAP_GHOST))
    {
        cache_hit++;
        opagemap[blkno].map_age++;
        if(opagemap[blkno].map_status == MAP_GHOST)
        {
            DFTL_Hit_Ghost_CMT(blkno);
        }
        else if(opagemap[blkno].map_status == MAP_REAL) 
        {
            DFTL_Hit_Real_CMT(blkno);
        }
        else {
            printf("forbidden/shouldnt happen real =%d , ghost =%d\n",MAP_REAL,MAP_GHOST);
        }
    }

    //2. opagemap not in SRAM 
    else
    {
        //if map table in SRAM is full
        DFTL_Real_Cmt_Full();

        //read entry into real
        flash_hit++;
        // read from 2nd mapping table
        send_flash_request(((blkno-page_num_for_2nd_map_table)/MAP_ENTRIES_PER_PAGE)*4, 4, 1, 2);  
        send_flash_request(((blkno-page_num_for_2nd_map_table)/MAP_ENTRIES_PER_PAGE)*4, 4, 1, 2); 
        translation_read_num++;
        opagemap[blkno].map_status = MAP_REAL;
        opagemap[blkno].map_age = opagemap[real_max].map_age + 1;
        real_max = blkno;
        MAP_REAL_NUM_ENTRIES++;
        
        pos = find_free_pos(real_arr,MAP_REAL_MAX_ENTRIES);
        real_arr[pos] = blkno;
    }

    //comment out the next line when using cache
    if(operation==0){
        write_count++;
        opagemap[blkno].update = 1;
    }
    else
        read_count++;

    send_flash_request(blkno*4, 4, operation, 1); 
    blkno++;
    (*pageno)=blkno;
    (*req_size)=cnt;
}

void DFTL_init_arr()
{
    int i;
    for( i= 0; i< MAP_GHOST_MAX_ENTRIES;i++){
        ghost_arr[i] = -1;
    }

    for( i = 0; i < MAP_REAL_MAX_ENTRIES; i++) {
        real_arr[i] = -1;
    }
    MAP_REAL_NUM_ENTRIES=0;
    MAP_GHOST_NUM_ENTRIES=0;
}

void DFTL_Hit_Ghost_CMT(int blkno)
{
    int pos_ghost=-1,pos_real=-1;

    if ( real_min == -1 ) {
        real_min = 0;
        find_real_min();
    }    
    if(opagemap[real_min].map_age <= opagemap[blkno].map_age) 
    {
        find_real_min();  // probably the blkno is the new real_min alwaz
        opagemap[blkno].map_status = MAP_REAL;
        opagemap[real_min].map_status = MAP_GHOST;

        pos_ghost = search_table(ghost_arr,MAP_GHOST_MAX_ENTRIES,blkno);
        ghost_arr[pos_ghost] = -1;
        
        pos_real = search_table(real_arr,MAP_REAL_MAX_ENTRIES,real_min);
        real_arr[pos_real] = -1;

        real_arr[pos_real]  = blkno; 
        ghost_arr[pos_ghost] = real_min; 
    }
}

void DFTL_Hit_Real_CMT(int blkno)
{
    if ( real_max == -1 ) {
        real_max = 0;
        find_real_max();
        printf("Never happend\n");
    }

    if(opagemap[real_max].map_age <= opagemap[blkno].map_age){
        real_max = blkno;
    }  
}

void DFTL_Real_Cmt_Full()
{
    int pos=-1;
    if((MAP_REAL_MAX_ENTRIES - MAP_REAL_NUM_ENTRIES) == 0)
    {
        DFTL_Ghost_Cmt_Full();

        //evict one entry from real cache to ghost cache 
        MAP_REAL_NUM_ENTRIES--;
        find_real_min();
        opagemap[real_min].map_status = MAP_GHOST;
        pos = search_table(real_arr,MAP_REAL_MAX_ENTRIES,real_min);
        real_arr[pos]=-1;
        pos = find_free_pos(ghost_arr, MAP_GHOST_MAX_ENTRIES);
        ghost_arr[pos] = real_min;
        MAP_GHOST_NUM_ENTRIES++;
    }
}


void DFTL_Ghost_Cmt_Full()
{
    int ghost_min=-1, pos=-1;

    //evict one entry from ghost cache to DRAM or Disk, delay = DRAM or disk write, 1 oob write for invalidation 
    if((MAP_GHOST_MAX_ENTRIES - MAP_GHOST_NUM_ENTRIES) == 0)
    { 
        ghost_min = find_min_ghost_entry();
        evict++;//ymb

        if(opagemap[ghost_min].update == 1) {
            update_reqd++;//ymb
            opagemap[ghost_min].update = 0;
            // read from 2nd mapping table then update it
            send_flash_request(((ghost_min-page_num_for_2nd_map_table)/MAP_ENTRIES_PER_PAGE)*4, 4, 1, 2);  
            translation_read_num++; 
            // write into 2nd mapping table 
            send_flash_request(((ghost_min-page_num_for_2nd_map_table)/MAP_ENTRIES_PER_PAGE)*4, 4, 0, 2);   
            translation_write_num++;
        } 
        opagemap[ghost_min].map_status = MAP_INVALID;
        opagemap[ghost_min].map_age = 0;
        MAP_GHOST_NUM_ENTRIES--;

        pos = search_table(ghost_arr,MAP_GHOST_MAX_ENTRIES,ghost_min);
        ghost_arr[pos]=-1;
    }
}

void WRFTL_Scheme(int *pageno,int *req_size,int operation)
{
    int blkno=(*pageno),cnt=(*req_size);
    int ghost_min=-1,pos=-1;   //zan shi mei yong 
    Node *Temp;
    //初始化
    if (init_flag==0)
    {
        // CMT value size is 32KB include entry(512 in 2Kpage)
        // 这里real_arr当作WCMT WCMT 40KB －> 10240
        // 把ghost_arr当作RCMT
        // ghost_arr is 24KB ,6144 entries

        MAP_REAL_MAX_ENTRIES=6552;
        MAP_GHOST_MAX_ENTRIES=1640;



        real_arr=(int *)malloc(sizeof(int)*MAP_REAL_MAX_ENTRIES);
        ghost_arr=(int *)malloc(sizeof(int)*MAP_GHOST_MAX_ENTRIES);
        //WCMT的优先置换区
        WRFTL_Window_Size=(int)MAP_REAL_MAX_ENTRIES*WRFTL_Tau;
        WRFTL_Head=CreateList();
        WRFTL_init_arr();
        //request_cnt = rqst_cnt;
        init_flag=1;
    }
    
    /*******************正式进入仿真运行******************/
    operation_time++; //师兄是放到每个操作里面去，但是直接外面统计，应当更方便
    //req_entry hit in WCMT
    if(opagemap[blkno].map_status==MAP_REAL){
        if(ListLength(WRFTL_Head)!=MAP_REAL_NUM_ENTRIES){ //debug
            printf("before WRFTL hit in WCMT error, ListLength is %d, real_arr size is %d\n", ListLength(WRFTL_Head),MAP_REAL_NUM_ENTRIES);
            assert(0);
        }
        WRFTL_Hit_WCMT(blkno, operation);
        if(ListLength(WRFTL_Head)!=MAP_REAL_NUM_ENTRIES){ //debug
            printf("after WRFTL hit in WCMT error, ListLength is %d, real_arr size is %d\n", ListLength(WRFTL_Head),MAP_REAL_NUM_ENTRIES);
            assert(0);
        }
        //数据统计
        rqst_cnt++;
        cache_scmt_hit++;
        blkno++;
    }
    // req_entry hit in RCMT
    else if (opagemap[blkno].map_status==MAP_GHOST){
        //write
        if(operation==0){
            WRFTL_Move_RCMT2WCMT(blkno, operation);
            //debug
            if(ListLength(WRFTL_Head)!=MAP_REAL_NUM_ENTRIES){
                printf(" after WRFTL_Move_RCMT2WCMT error,ListLength is %d,real_arr size is %d\n",ListLength(WRFTL_Head),MAP_REAL_NUM_ENTRIES);
                assert(0);
            }
        }
        //read
        else{
            WRFTL_Move_RCMT2MRU(blkno, operation);

        }
        //数据统计
        rqst_cnt++;
        cache_slcmt_hit++;
        blkno++;
    }
    // req_entry miss in CMT
    else{
        //预取
        if(operation==0)
            WRFTL_Write_Pre_Load(&blkno, &cnt);
        else
            WRFTL_Read_Pre_Load(&blkno, &cnt);
    }
    (*pageno)=blkno,(*req_size)=cnt;
    
}


void WRFTL_init_arr()
{
    int i;
    for( i = 0; i < MAP_REAL_MAX_ENTRIES; i++) {
        real_arr[i] = -1;
    }
    for( i= 0; i < MAP_GHOST_MAX_ENTRIES; i++) {
        ghost_arr[i] = -1;
    }
    MAP_REAL_NUM_ENTRIES = 0;
    MAP_GHOST_NUM_ENTRIES = 0;
}

/*********************
* 请求命中WCMT
*/
void WRFTL_Hit_WCMT(int blkno, int operation)
{
    int hot_flag=0, Len=0;
    Node *temp;
    opagemap[blkno].map_status=MAP_REAL;
    opagemap[blkno].map_age=operation_time;

    //操作链表的LRU
    //temp=SearchLPNInList(blkno, WRFTL_Head);
    temp = IsHotLPNInList(blkno, WRFTL_Head, &Len);
    
    if(temp==NULL){
        printf("error in ADFTL_Hit_R_CMT,can not find blkno %d in List\n",blkno);
        assert(0);
    }
    else{
        //move node
        InsertNodeInListMRU(temp,WRFTL_Head);
    }
    if(operation==0){
        write_count++;
        opagemap[blkno].update = 1;
    }
    else
        read_count++;

    //send_flash_request(blkno*4, 4, operation, 1);
    if(Len > (int)(MAP_REAL_MAX_ENTRIES * hot_ratio)){
        send_flash_request(blkno*4, 4, operation, 1);
    }
    else{
        send_flash_request(blkno*4, 4, operation, 0);
    }
}

/**************************
* 读请求命中RMCT
* 映射信息迁移至RCMT的MRU
*/
void WRFTL_Move_RCMT2MRU(int blkno, int operation)
{
    //ghost_arr不变，map_age改变就相当于是迁移至MRU
    opagemap[blkno].map_age=operation_time;
 
    if(operation==0){
        write_count++;
        opagemap[blkno].update = 1;
        printf("write operation happend in RCMT, LPN=%d\n",blkno);
        assert(0);
    }
    else
        read_count++;
    //ghost_arr不进行移动操作
    send_flash_request(blkno*4, 4, operation, 1);
}

/************************
* 写请求命中RCMT
* 将映射信息从RCMT中迁移至WCMT的MRU位置
*/
void WRFTL_Move_RCMT2WCMT(int blkno, int operation)
{
    int free_pos=-1;
    int r_pos=-1;
    WRFTL_WCMT_Is_Full(0);
    //从RCMT中剔除
    r_pos=search_table(ghost_arr, MAP_GHOST_MAX_ENTRIES, blkno);
    if(r_pos==-1){
        printf("can not find blkno :%d in ghost_arr\n",blkno);
        assert(0);
    }
    ghost_arr[r_pos]=-1;
    MAP_GHOST_NUM_ENTRIES--;
    //加入WCMT中
    free_pos=find_free_pos(real_arr, MAP_REAL_MAX_ENTRIES);
    if(free_pos==-1){
        printf("can not find free_pos in real_arr\n");
        assert(0);
    }
    real_arr[free_pos]=blkno;
    opagemap[blkno].map_status=MAP_REAL;
    opagemap[blkno].map_age=operation_time;
    //应只有写操作
    if(operation==0){
        write_count++;
        opagemap[blkno].update = 1;
    }
    else
        read_count++;
    //链表操作
    AddNewLPNInMRU(blkno, WRFTL_Head);
    MAP_REAL_NUM_ENTRIES++;
    if(MAP_REAL_NUM_ENTRIES>MAP_REAL_MAX_ENTRIES){
        printf("real_arr overflow (WRFTL_Move_RCMT2WCMT) lpn=%d\n",blkno);
        assert(0);
    }
    send_flash_request(blkno*4, 4, operation, 1);
}

/************************
* 批量预取读写
* 读预取到RCMT
* 写预取到WCMT
* 读写都按照预取大小进行预取
*/
void WRFTL_Write_Pre_Load(int *pageno, int *req_size)
{
    int blkno=(*pageno),cnt=(*req_size);
    int pos=-1,free_pos=-1;
    int temp_num=0;
    int indexofarr=0;
    Node *temp;

    flash_hit++;  // q:ymb flash_hit 是什么？
    send_flash_request(((blkno-page_num_for_2nd_map_table)/MAP_ENTRIES_PER_PAGE)*4, 4, 1, 2);
    translation_read_num++;
    // 对于写请求，不采取预取策略
    for(indexofarr =0; (cnt+1>0);indexofarr++)//NUM_ENTRIES_PER_TIME在这里表示一次加载多个映射表信息
    {
        WRFTL_WCMT_Is_Full();
        
        // 若是超过了此翻译页的范围，则读取下一个翻译页
        if(((blkno-page_num_for_2nd_map_table)%MAP_ENTRIES_PER_PAGE) ==0 ){
            flash_hit++; 
            send_flash_request((((blkno-page_num_for_2nd_map_table)/MAP_ENTRIES_PER_PAGE)+1)*4, 4, 1, 2);
            translation_read_num++;
        }
        if(not_in_cache(blkno)==0){   // not in cache
            //映射信息加载
            opagemap[blkno].map_status=MAP_REAL;
            opagemap[blkno].map_age=operation_time;
            opagemap[blkno].update=1;
            MAP_REAL_NUM_ENTRIES++;
            AddNewLPNInMRU(blkno, WRFTL_Head);

            free_pos=find_free_pos(real_arr, MAP_REAL_MAX_ENTRIES);
            if(free_pos==-1){
                printf("can not find free_pos in real_arr\n");
                assert(0);
            }
            real_arr[free_pos]=blkno;
            //对请求进行处理
            write_count++;
            send_flash_request(blkno*4, 4, 0, 1);

        }
        else if(not_in_cache(blkno)==1){  // in real arr
            WRFTL_Hit_WCMT(blkno, 0);
            cache_scmt_hit++;
        }  
        else{  // in ghost arr
            WRFTL_Move_RCMT2WCMT(blkno, 0);
            cache_slcmt_hit++;
        }
        cnt--;
        operation_time++;
        blkno++;
        rqst_cnt++;
        //debug
        if(MAP_REAL_NUM_ENTRIES > MAP_REAL_MAX_ENTRIES){
            printf("The WCMT is overflow\n");
            assert(0);
        }
    }
    *req_size=cnt;
    *pageno=blkno;
}

void WRFTL_Read_Pre_Load(int *pageno, int *req_size)
{
    int blkno=(*pageno),cnt=(*req_size);
    int pos=-1,free_pos=-1;
    int temp_num=0;
    int indexofarr=0;
    Node *temp;

    //RCMT 预取
    flash_hit++;  // q:ymb flash_hit 是什么？
    send_flash_request(((blkno-page_num_for_2nd_map_table)/MAP_ENTRIES_PER_PAGE)*4, 4, 1, 2);;
    translation_read_num++;
    // 读取采用预取操作，取MAX{cnt,NUM_ENTRIES_PER_TIME} 
    //for(indexofarr =0; (cnt+1>0) || indexofarr < NUM_ENTRIES_PER_TIME;indexofarr++)//NUM_ENTRIES_PER_TIME在这里表示一次加载4个映射表信息
    for(indexofarr =0; (cnt+1>0) || indexofarr < NUM_ENTRIES_PER_TIME; indexofarr++)
    {
        WRFTL_RCMT_Is_Full();
        if((cnt+1)>0){
            // 若是超过了此翻译页的范围，则读取下一个翻译页
            if(((blkno-page_num_for_2nd_map_table)%MAP_ENTRIES_PER_PAGE) ==0 ) {
                flash_hit++; 
                send_flash_request((((blkno-page_num_for_2nd_map_table)/MAP_ENTRIES_PER_PAGE)+1)*4, 4, 1, 2);
                translation_read_num++;
            }
            if(not_in_cache(blkno) == 0){
                //预取映射信息
                opagemap[blkno].map_status=MAP_GHOST;
                opagemap[blkno].map_age=operation_time;
                opagemap[blkno].update=0;
                MAP_GHOST_NUM_ENTRIES++;
                // 如果是取MIN{cnt,NUM_ENTRIES_PER_TIME}，直接cnt--会有问题，会出现cnt>NUM_..时，少一次

                free_pos=find_free_pos(ghost_arr, MAP_GHOST_MAX_ENTRIES);
                if(free_pos==-1){
                    printf("can not find free_pos in real_arr\n");
                    assert(0);
                }
                ghost_arr[free_pos]=blkno;
                read_count++;
                send_flash_request(blkno*4, 4, 1, 1);
            }
            else if(not_in_cache(blkno) == 1){   // in real arr
                WRFTL_Hit_WCMT(blkno, 1);
                cache_scmt_hit++;
            }
            else{   // in ghost arr
                WRFTL_Move_RCMT2MRU(blkno, 1);
                cache_slcmt_hit++;
            }
            cnt--;
            operation_time++;
            blkno++;
            rqst_cnt++;
            //debug
            if(MAP_GHOST_NUM_ENTRIES > MAP_GHOST_MAX_ENTRIES){
                printf("The RCMT is overflow\n");
                assert(0);
            }
        }
        else{
            // 若是超过了此翻译页的范围，则读取下一个翻译页
            if(((blkno-page_num_for_2nd_map_table)%MAP_ENTRIES_PER_PAGE) ==0 ) {
                flash_hit++; 
                send_flash_request((((blkno-page_num_for_2nd_map_table)/MAP_ENTRIES_PER_PAGE)+1)*4, 4, 1, 2);
                translation_read_num++;
            }
            if(not_in_cache(blkno) == 0){
                //预取映射信息
                opagemap[blkno].map_status=MAP_GHOST;
                opagemap[blkno].map_age=operation_time;
                opagemap[blkno].update=0;
                operation_time++;
                MAP_GHOST_NUM_ENTRIES++;

                free_pos=find_free_pos(ghost_arr, MAP_GHOST_MAX_ENTRIES);
                if(free_pos==-1){
                    printf("can not find free_pos in real_arr\n");
                    assert(0);
                }
                ghost_arr[free_pos]=blkno;
                //read_count++;
                //由于预取
                //send_flash_request(blkno*4, 4, 1, 1);
                blkno++;
                //rqst_cnt++;
            }
        }
        
    }
    *req_size=cnt;
    *pageno=blkno;
}

/******************************
* 加载单个写请求到WCMT
*/
void WRFTL_Load_Entry2WCMT(int blkno, int operation)
{
    int free_pos=-1;
    flash_hit++;
    //read MVPN page
    send_flash_request(((blkno-page_num_for_2nd_map_table)/MAP_ENTRIES_PER_PAGE)*4, 4, 1, 2);;   // read from 2nd mapping table
    translation_read_num++;
    opagemap[blkno].map_status=MAP_REAL;
    opagemap[blkno].map_age=operation_time;
    free_pos=find_free_pos(real_arr, MAP_REAL_MAX_ENTRIES);
    if(free_pos==-1){
        printf("can not find free pos in real_arr\n");
        assert(0);
    }
    real_arr[free_pos]=blkno;
    //链表操作
    AddNewLPNInMRU(blkno, WRFTL_Head);
    MAP_REAL_NUM_ENTRIES++;
    //write  data page
    if(operation==0){
        write_count++;
        opagemap[blkno].update = 1;
    }
    else
        read_count++;

    send_flash_request(blkno*4, 4, operation, 1);

    // debug test
    if(opagemap[blkno].map_status!=MAP_REAL){
        printf("not set opagemap flag\n");
        assert(0);
    }
    if(search_table(real_arr,MAP_REAL_MAX_ENTRIES,blkno)==-1){
        printf("not play lpn-entry:%d into CMT\n",blkno);
        assert(0);
    }

    if(SearchLPNInList(blkno,WRFTL_Head)==NULL){
    printf("not Add blkno %d into List\n",blkno);
    assert(0);
    }

    if(ListLength(WRFTL_Head)!=MAP_REAL_NUM_ENTRIES){
    printf("List Length is %d and real_arr num is %d\n",ListLength(WRFTL_Head),MAP_REAL_NUM_ENTRIES);
    assert(0);
    }

}
/**********************************
    * 加载耽搁读请求到RCMT
    */
void WRFTL_Load_Entry2RCMT(int blkno, int operation)
{
    int free_pos=-1;
    flash_hit++;
    // read MVPN page
    send_flash_request(((blkno-page_num_for_2nd_map_table)/MAP_ENTRIES_PER_PAGE)*4, 4, 1, 2);  // read from 2nd mapping table
    translation_read_num++;
    opagemap[blkno].map_status = MAP_GHOST;

    opagemap[blkno].map_age=operation_time;
    free_pos=find_free_pos(ghost_arr,MAP_GHOST_MAX_ENTRIES);
    if(free_pos==-1){
        printf("can not find free pos in ghost_arr\n");
        assert(0);
    }
    ghost_arr[free_pos]=blkno;
    MAP_GHOST_NUM_ENTRIES++;
    // read data page
    if(operation==0){
        write_count++;
        opagemap[blkno].update = 1;
    }
    else
        read_count++;

    send_flash_request(blkno*4, 4, operation, 1);

    // debug test
    if(opagemap[blkno].map_status!=MAP_GHOST){
        printf("not set opagemap flag\n");
        assert(0);
    }
    if(search_table(ghost_arr,MAP_GHOST_MAX_ENTRIES,blkno)==-1){
        printf("not play lpn-entry:%d into CMT\n",blkno);
        assert(0);
    }
}

/**********************************
    * 对WCMT进行是否满对判断
    * flag表示是否是预取的判断
    * flag＝1表示预取，需要判断空间是否小于预取个数个
    */
void WRFTL_WCMT_Is_Full()
{
    //判断是否满
    if(MAP_REAL_NUM_ENTRIES-MAP_REAL_MAX_ENTRIES == 0){
        WRFTL_Remove_Entry_In_WCMT();
    }
}

/********************************
* 对WCMT映射信息剔除的封装
*/
void WRFTL_Remove_Entry_In_WCMT()
{
    int Victim_pos=-1, find_free_pos=-1, curr_lpn;
    int temp_num = 0;
    int indexold = 0;
    Node *Temp;

    Victim_pos=WRFTL_Find_Victim_In_WCMT();  //此函数实现优先置换区干净页的优先剔除
    curr_lpn=real_arr[Victim_pos];

    if(opagemap[curr_lpn].update!=0){
        //表明优先置换区没有干净页，选择zang ye，聚簇剔除
        //先将受害页剔除，之后与同簇数据一同回写
        opagemap[curr_lpn].map_status=MAP_INVALID;
        opagemap[curr_lpn].map_age=0;
        opagemap[curr_lpn].update=0;
        real_arr[Victim_pos]=-1;
        Temp=DeleteLRUInList(WRFTL_Head);
        if(Temp->lpn_num!=curr_lpn){
            printf("delete lru arr Temp->lpn %d not equal curr-lpn %d\n",Temp->lpn_num,curr_lpn);
            assert(0);
        }
        MAP_REAL_NUM_ENTRIES--;
        //整簇回写，数据统计
        //用maxentry代表待剔除的簇
        maxentry=(curr_lpn-page_num_for_2nd_map_table)/MAP_ENTRIES_PER_PAGE;
        send_flash_request(maxentry*4,4,1,2);
        translation_read_num++;
        send_flash_request(maxentry*4,4,0,2);
        translation_write_num++;

        //real_arr数组里面存的是lpn,将翻译页关联的映射项全部置为干净
        for(indexold = 0;indexold < MAP_REAL_MAX_ENTRIES; indexold++){
            if(((real_arr[indexold]-page_num_for_2nd_map_table)/MAP_ENTRIES_PER_PAGE) == maxentry){
                opagemap[real_arr[indexold]].update = 0;
            }
        }
    }
    //仅干净页剔除，无需回写操作
    else{
        opagemap[curr_lpn].map_status=MAP_INVALID;
        opagemap[curr_lpn].map_age=0;
        opagemap[curr_lpn].update=0;
        real_arr[Victim_pos]=-1;
        //链表操作，删除被置换节点
        Temp=SearchLPNInList(curr_lpn, WRFTL_Head);
        DeleteNodeInList(Temp, WRFTL_Head);
        if(Temp->lpn_num!=curr_lpn){
        printf("delete lru arr Temp->lpn %d not equal curr-lpn %d\n",Temp->lpn_num,curr_lpn);
        assert(0);
        }
        MAP_REAL_NUM_ENTRIES--;
    }
}


/********************************
* 对RCMT进行判断是否满
* flag与WCMT功能一致
*/
void WRFTL_RCMT_Is_Full()
{
    int ghost_min=-1, pos=-1;
    int temp_num=0;
    if(MAP_GHOST_NUM_ENTRIES-MAP_GHOST_MAX_ENTRIES == 0){
        ghost_min=find_min_ghost_entry();
        if(opagemap[ghost_min].update ==1){
            printf("RCMT have dirty entry\n");
            assert(0);
        }
        opagemap[ghost_min].map_status=MAP_INVALID;
        opagemap[ghost_min].map_age=0;

        //evict one entry from ghost cache 
        MAP_GHOST_NUM_ENTRIES--;
        pos=search_table(ghost_arr,MAP_GHOST_MAX_ENTRIES,ghost_min);
        if(pos==-1){
            printf("can not find ghost_min:%d  in ghost_arr\n",ghost_min);
            assert(0);
        }
        ghost_arr[pos]=-1;
    }
}


/***********************************
    * 通过双链表，在WCMT中找到受害页
    */
int WRFTL_Find_Victim_In_WCMT()
{
    Node *Temp;
    int i,pos_index,Victim_index,curr_lpn,clean_flag=0;
    //  从尾部进行扫描,优先找到干净项进行删除
    Temp=WRFTL_Head;
    for(i=0;i<WRFTL_Window_Size;i++){
        Temp=Temp->pre;
        curr_lpn=Temp->lpn_num;
        if(opagemap[curr_lpn].update==0 && opagemap[curr_lpn].map_status==MAP_REAL){
            clean_flag=1;
        break;
        }
    }

    if(clean_flag==0){
    //      选择LRU位置的脏映射项
        curr_lpn=WRFTL_Head->pre->lpn_num;
        Victim_index=search_table(real_arr,MAP_REAL_MAX_ENTRIES,curr_lpn);
        if(Victim_index==-1){
        printf("can not find LRU pos lpn %d in real_arr\n",curr_lpn);
        assert(0);
        }

    }else{
    //    选择窗口内的干净页映射项
        if(opagemap[curr_lpn].update!=0){
        printf("error opagemap[%d]->update can not be update\n",curr_lpn);
        assert(0);
        }
        Victim_index=search_table(real_arr,MAP_REAL_MAX_ENTRIES,curr_lpn);
        if(Victim_index==-1){
        printf("can not find LRU pos lpn %d in real_arr\n",curr_lpn);
        assert(0);
        }
    }
    return Victim_index;
}

//~处理之后所有的请求映射项都不在CMT中才预取 
int not_in_cache(unsigned int pageno)
{
    int flag = 0;
    
    if((opagemap[pageno].map_status == 0) || (opagemap[pageno].map_status == MAP_INVALID))
        flag=0;
    else if(opagemap[pageno].map_status == MAP_REAL)
        flag=1;
    else
        flag=2;
    
    return flag;
}



/*************************************************************
*              author:ymb        PPFTL
* *********************************************************/

void PPFTL_Scheme(int *pageno,int *req_size,int operation)
{
    int blkno=(*pageno),cnt=(*req_size);

    rqst_cnt++;
    if(operation==0){
        write_count++;
    }
    else
        read_count++;
    send_flash_request(blkno*4, 4, operation, 1);
    blkno++;
    //syn-value
    (*pageno)=blkno,(*req_size)=cnt;
}


