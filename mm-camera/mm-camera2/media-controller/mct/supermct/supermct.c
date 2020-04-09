/* mct_supermct.c
 *
 * This file contains the media controller implementation. All commands coming
 * from the server arrive here first. There is one media controller per
 * session.
 *
 *Copyright (c) 2016-2017 Qualcomm Technologies, Inc.
  * All Rights Reserved.
  * Confidential and Proprietary - Qualcomm Technologies, Inc.
  */

#include "cam_ker_headers.h"
#include "media_controller.h"
#include "mct_controller.h"
#include "mct_profiler.h"
#include "mct_pipeline.h"
#include "mct_bus.h"
#include "cam_intf.h"
#include "camera_dbg.h"
#include "mm_camera_shim.h"
#include <cutils/properties.h>
#include <server_debug.h>
#include "supermct.h"
#include <pthread.h>

static supermct_t *g_supermct = NULL;

/**
 *      static/Internal functions
 *
**/

/** supermct_init:
  *    This function initialize supermct
  *
  *
  *  Return: TRUE, if sucessful
  *         FALSE, if error
  **/
static boolean supermct_init()
{
  g_supermct = malloc(sizeof(supermct_t));
  if (!g_supermct) {
    CLOGE(CAM_MCT_MODULE, "Malloc failed");
    return FALSE;
  }
  memset(g_supermct, 0, sizeof(supermct_t));

  pthread_mutex_init(&g_supermct->sof_q_mutex, NULL);

  pthread_cond_init(&g_supermct->supermct_cond, NULL);
  pthread_mutex_init(&g_supermct->supermct_mutex, NULL);
  pthread_mutex_init(&g_supermct->update_sof_id_mutex, NULL);

  g_supermct->sof_queue = malloc(sizeof(mct_queue_t));
  if (!g_supermct->sof_queue) {
    CLOGE(CAM_MCT_MODULE, "Malloc failed");
    return FALSE;
  }
  mct_queue_init(g_supermct->sof_queue);

  return TRUE;
}

/** supermct_deinit:
 *    This function deinit supermct
 *
 *
 *  Return: TRUE, if sucessful
 *         FALSE, if error
 **/
static void supermct_deinit(supermct_t *l_supermct)
{
  pthread_cond_destroy(&l_supermct->supermct_cond);
  pthread_mutex_destroy(&l_supermct->supermct_mutex);
  pthread_mutex_destroy(&l_supermct->sof_q_mutex);
  pthread_mutex_destroy(&l_supermct->update_sof_id_mutex);

  if (!MCT_QUEUE_IS_EMPTY(l_supermct->sof_queue)) {
    mct_queue_free(l_supermct->sof_queue);
  } else {
    free(l_supermct->sof_queue);
    l_supermct->sof_queue = NULL;
  }

  free(g_supermct);
  g_supermct = NULL;
}

/** supermct_append:
 *    This function add MCT to super mct list
 *
 *    @pipeline: Passing pipeline. need to find parent of pipeline
 *    @super_mct: Supermct as input to add MCT in supermct list
 *
 *  Return: TRUE, if sucessful
 *         FALSE, if error
 **/
static boolean supermct_append(mct_pipeline_t *pipeline, supermct_t *l_supermct)
{
  mct_controller_t *mct = NULL;

  if (!pipeline) {
    CLOGE(CAM_MCT_MODULE,"pipeline is NULL");
    return FALSE;
  }

  /* Get pipeline's parent ie MCT */
  mct = MCT_CAST((MCT_PIPELINE_PARENT(pipeline->object))->data);

  if (!mct) {
    CLOGE(CAM_MCT_MODULE,"MCT is NULL");
    return FALSE;
  }

  CLOGL(CAM_MCT_MODULE,"Number of SuperMCT child %d\n",
    SUPERMCT_NUM_CHILDREN(l_supermct));

  /* Supermct's child is MCT*/
  SUPERMCT_LOCK(l_supermct);
  SUPERMCT_CHILDREN(l_supermct) = mct_list_append(SUPERMCT_CHILDREN(l_supermct),
    mct, NULL, NULL);
  if (!MCT_OBJECT_CHILDREN(l_supermct)) {
    CLOGE(CAM_MCT_MODULE, "Error configuring single module");
    SUPERMCT_UNLOCK(l_supermct);
    return FALSE;
  }
  (SUPERMCT_NUM_CHILDREN(l_supermct))++;
  SUPERMCT_UNLOCK(l_supermct);

  /* Add supermct as MCT's parent */
  MCT_LOCK(mct);
  MCT_PARENT(mct) = mct_list_append(MCT_PARENT(mct), l_supermct, NULL, NULL);
  if (!MCT_PARENT(mct)) {
    CLOGE(CAM_MCT_MODULE, "Couldn't append pipeline as MCT's child");
    MCT_UNLOCK(mct);
    return FALSE;
  }
  MCT_UNLOCK(mct);

  return TRUE;
}


/** supermct_remove:
 *    This function remove MCT from super mct list
 *
 *    @super_mct: Supermct as input to add MCT in supermct list
 *
 *  Return: TRUE, if sucessful
 *         FALSE, if error
 **/
static boolean supermct_remove(supermct_t *l_supermct)
{
  /* Remove all supermct from MCT */
  /* Remove all MCTs from supermct */
  SUPERMCT_LOCK(l_supermct);
  if (SUPERMCT_CHILDREN(l_supermct)) {
    mct_list_free_list(SUPERMCT_CHILDREN(l_supermct));
  }
  SUPERMCT_CHILDREN(l_supermct) = NULL;
  SUPERMCT_NUM_CHILDREN(l_supermct) = 0;
  SUPERMCT_UNLOCK(l_supermct);

  return TRUE;
}


/** supermct_wakeup_mct:
 *    This function wakes ups MCT when required
 *
 *    @bus: Bus payload structre as input,
 *          As it needs to wake up MCT SOF thread
 *
 *    @msg: Bus message payload structre as input,
 *          This SOF bus message.
 *
 *    @bus_msg_type: Bus message type is internal bus message for
            with parameter or without parameter configuration.
 *
 *  Return: TRUE, if sucessful
           FALSE, if error
 **/
static boolean supermct_wakeup_mct(mct_bus_t *bus, void *msg,
  mct_bus_msg_type_t bus_msg_type)
{
  mct_bus_msg_t *bus_msg = msg;

  /*
    Internal Bus messages
    MCT_BUS_INTMSG_PROC_SOF_WO_PARAM
    MCT_BUS_INTMSG_PROC_SOF_W_PARAM
  */

  bus_msg->type = bus_msg_type;
  pthread_mutex_lock(&bus->priority_q_lock);
  mct_queue_push_tail(bus->priority_queue, bus_msg);
  pthread_mutex_unlock(&bus->priority_q_lock);

  pthread_mutex_lock(bus->mct_mutex);
  pthread_cond_signal(bus->mct_cond);
  pthread_mutex_unlock(bus->mct_mutex);

  return TRUE;
}

/** supermct_print_nodes:
 *    This function prints node in the queue.
 *     This is used for debugging purpose.
 *
 *    @qdata: This point to data of list
 *    @userdata: User data or payload to operate on both nodes
 *
 *  Return: VOID
 **/
boolean supermct_print_nodes(void* qdata, void* userdata)
{
  CLOGL(CAM_MCT_MODULE, "qdata=%p userdata=%p", qdata, userdata);
  return TRUE;
}

/** supermct_update_zone:
 *    This function updates Dual camera zones
 *    Dual Camera Zones:
 *           Wide:   Primary: Awake,  Auxiliary: Asleep
 *           Dual:   Primary: Awake,  Auxiliary: Awake
 *           Tele:   Primary: Asleep, Auxiliary: Awake
 *
 *    @d1: This is first node of list
 *    @d2: This is second node of list
 *
 *    @user_data: User data or payload to operate on both nodes
 *
 *  Return: VOID
 **/
static void supermct_update_zone(void *d1, void *d2, const void *user_data)
{
  mct_controller_t *mct1 = (mct_controller_t *)d1;
  mct_controller_t *mct2 = (mct_controller_t *)d2;
  mct_pipeline_t *pipeline1 = NULL;
  mct_pipeline_t *pipeline2 = NULL;
  mct_pipeline_t *current_pipeline = NULL;
  mct_bus_msg_isp_sof_t *isp_sof_bus_msg = NULL;
  boolean book_keeping_flag = FALSE;

  if (!mct1 || !mct2) {
    CLOGE(CAM_MCT_MODULE, "mct1 or mct2 is NULL");
    return;
  }

  pipeline1 = mct1->pipeline;
  pipeline2 = mct2->pipeline;

  supermct_t *l_supermct = (supermct_t *)user_data;

  if (!l_supermct) {
    CLOGE(CAM_MCT_MODULE, "supermct is NULL");
    return;
  }

  if (!pipeline1 || !pipeline2) {
    CLOGE(CAM_MCT_MODULE, "Pipeline1 or pipeline2 is NULL");
    return;
  }

  CLOGL(CAM_MCT_MODULE,"P1: session %d, module_hw_state %d",
    pipeline1->session, pipeline1->module_hw_state);
  CLOGL(CAM_MCT_MODULE,"P2: session %d, module_hw_state %d",
    pipeline2->session, pipeline2->module_hw_state);

  if ((MCT_MODULE_STATE_AWAKE == pipeline1->module_hw_state) &&
      (MCT_MODULE_STATE_AWAKE == pipeline2->module_hw_state)) {
    l_supermct->supermct_zone = SUPERMCT_DUAL_ZONE;
  } else if ((MCT_MODULE_STATE_AWAKE == pipeline1->module_hw_state) &&
      (MCT_MODULE_STATE_ASLEEP == pipeline2->module_hw_state)) {
    l_supermct->supermct_zone = SUPERMCT_WIDE_ZONE;
  } else if ((MCT_MODULE_STATE_ASLEEP == pipeline1->module_hw_state) &&
      (MCT_MODULE_STATE_AWAKE == pipeline2->module_hw_state)) {
    l_supermct->supermct_zone = SUPERMCT_TELE_ZONE;
  }
}

/** supermct_validate_params:
 *    This function validates the paremeter received from HAL
 *    It validates,
 *       parameter is for dual camera or not
 *      IS this first camera's sof or second camera SOF
 *      Is book keeping has been done by other camera or not
 *      Traverse through super parameters and update the parameters
 *      It peek in to super parameters without pulling out super parameters.
 *
 *    @d1: This is first node of list
 *    @d2: This is second node of list
 *
 *    @user_data1: Super_mct: User data or payload to operate on both nodes
 *    @user_data2: sof_msg: User data or payload to operate on both nodes
 *
 *  Return: VOID
 **/
static void supermct_validate_params(void *d1, void *d2,
  const void *user_data1, const void *user_data2)
{
  uint32_t queue_length1 = 0, queue_length2 = 0, min_queue_length = 0,idx = 0;
  mct_event_super_control_parm_t *superparam1 = NULL;
  mct_event_super_control_parm_t *superparam2 = NULL;
  mct_controller_t *mct1 = (mct_controller_t *)d1;
  mct_controller_t *mct2 = (mct_controller_t *)d2;
  mct_pipeline_t *pipeline1 = NULL;
  mct_pipeline_t *pipeline2 = NULL;
  mct_pipeline_t *current_pipeline = NULL;
  mct_bus_msg_isp_sof_t *isp_sof_bus_msg = NULL;
  boolean book_keeping_flag = FALSE;

  if (!mct1 || !mct2) {
    CLOGE(CAM_MCT_MODULE, "mct1 or mct2 is NULL");
    return;
  }

  pipeline1 = mct1->pipeline;
  pipeline2 = mct2->pipeline;

  supermct_t *l_supermct = (supermct_t *)user_data1;
  mct_bus_msg_t *sof_msg = (mct_bus_msg_t *)user_data2;

  if (!l_supermct) {
    CLOGE(CAM_MCT_MODULE, "supermct is NULL");
    return;
  }

  if (!pipeline1 || !pipeline2) {
    CLOGE(CAM_MCT_MODULE, "Pipeline1 or pipeline2 is NULL");
    return;
  }

  if (sof_msg->sessionid  == pipeline1->session) {
    current_pipeline = pipeline1;
    CLOGL(CAM_MCT_MODULE, "Current pipeline is 1");
  } else {
    current_pipeline = pipeline2;
    CLOGL(CAM_MCT_MODULE, "Current pipeline is 2");
  }

  isp_sof_bus_msg = (mct_bus_msg_isp_sof_t *)sof_msg->msg;

  supermct_lock_sof_update();

  superparam1= mct_queue_look_at_head(pipeline1->super_param_queue);
  superparam2= mct_queue_look_at_head(pipeline2->super_param_queue);

  if((pipeline1->hal_version == CAM_HAL_V3)
       || (pipeline2->hal_version == CAM_HAL_V3))
  {
    if(((!(!(superparam1))) && (!(!superparam2)))
        && ((!superparam1->is_sync_param)||(!superparam2->is_sync_param))){
      mct_bus_msg_type_t msg_type = MCT_BUS_INTMSG_PROC_SOF_W_PARAM;
        //If sync params are false need to avoid duplicate book-keepings.
      //1.If current_session has invalid frame_number or current session frame_number
      //is less then other session frame_number (other param also should have valid
      //param) then if last valid sof is < current frame id then wakeup mct with parm.
      mct_event_super_control_parm_t *c_parm = (current_pipeline == pipeline1)?
                                                  superparam1:superparam2;
      mct_event_super_control_parm_t *o_parm = (current_pipeline == pipeline1)?
                                                superparam2:superparam1;
      if((c_parm->frame_number == UINT32_MAX)
         ||((o_parm->frame_number != UINT32_MAX)
         && (c_parm->frame_number < o_parm->frame_number)
         && (g_supermct->last_valid_sof_id < isp_sof_bus_msg->frame_id)))
      {
        msg_type = MCT_BUS_INTMSG_PROC_SOF_W_PARAM;
      } else if((current_pipeline->aux_cam == true) && (o_parm->frame_number != UINT32_MAX)){
         //2.slave contain valid params but master don't that could be due to delay in master sof.
         msg_type = MCT_BUS_INTMSG_PROC_SOF_WO_PARAM;
      }else {
        //3. just compare current params with other session params wake-up mct if c_parm < o_parm.
        if((c_parm->frame_number < o_parm->frame_number)
        && (g_supermct->last_valid_sof_id < isp_sof_bus_msg->frame_id))
        {
            msg_type = MCT_BUS_INTMSG_PROC_SOF_W_PARAM;
        }else{
            msg_type = MCT_BUS_INTMSG_PROC_SOF_WO_PARAM;
        }
      }

      CLOGL(CAM_MCT_MODULE,"supermct process sof id %d %s param c_parm :%d o_parm:%d",
                                    isp_sof_bus_msg->frame_id,
                                    (msg_type == MCT_BUS_INTMSG_PROC_SOF_W_PARAM)? "with":"without",
                                    c_parm->frame_number, o_parm->frame_number);

      supermct_wakeup_mct(current_pipeline->bus, sof_msg, msg_type);
      supermct_unlock_sof_update();
      return;
    }

    //for invalid superparam send sof to mct.
    if((!superparam1 || (superparam1->frame_number == UINT32_MAX))
        || (!superparam2 || (superparam2->frame_number == UINT32_MAX))){
      //For synchronizing sof id's validate with last ack id.
      if(g_supermct->last_valid_sof_id < isp_sof_bus_msg->frame_id){
        CLOGL(CAM_MCT_MODULE,"supermct process sof id %d with param",
                                                isp_sof_bus_msg->frame_id);
        supermct_wakeup_mct(current_pipeline->bus, sof_msg,
            MCT_BUS_INTMSG_PROC_SOF_W_PARAM);
     }else {
        CLOGL(CAM_MCT_MODULE,"supermct process sof id %d without param",
                                                  isp_sof_bus_msg->frame_id);
        supermct_wakeup_mct(current_pipeline->bus, sof_msg,
        MCT_BUS_INTMSG_PROC_SOF_WO_PARAM);
     }
     supermct_unlock_sof_update();
     return;
    }
  }

  supermct_unlock_sof_update();
  if (!superparam1 || !superparam2) {
    CLOGL(CAM_MCT_MODULE, "All superparams are processed");
    supermct_wakeup_mct(current_pipeline->bus, sof_msg,
      MCT_BUS_INTMSG_PROC_SOF_W_PARAM);
    return;
  }

  if (!superparam1->is_sync_param) {
    CLOGL(CAM_MCT_MODULE, "Not sync param, no need to sync");
    supermct_wakeup_mct(current_pipeline->bus, sof_msg,
      MCT_BUS_INTMSG_PROC_SOF_W_PARAM);
    return;
  }

  if (!superparam2->is_sync_param) {
    CLOGL(CAM_MCT_MODULE, "Not sync param, no need to sync");
    supermct_wakeup_mct(current_pipeline->bus, sof_msg,
      MCT_BUS_INTMSG_PROC_SOF_W_PARAM);
    return;
  }

  if (superparam1->applying_frame_id != 0) {
    CLOGL(CAM_MCT_MODULE, "applying_frame_id already marked to [%d] Wakeup MCT1 SOF_th with Param",
      superparam1->applying_frame_id);
    supermct_wakeup_mct(current_pipeline->bus, sof_msg,
      MCT_BUS_INTMSG_PROC_SOF_W_PARAM);
     return;
  }

  if (superparam2->applying_frame_id != 0) {
    CLOGL(CAM_MCT_MODULE, "applying_frame_id already marked to [%d],Wakeup MCT2 SOF_th with Param",
     superparam2->applying_frame_id);
    supermct_wakeup_mct(current_pipeline->bus, sof_msg,
      MCT_BUS_INTMSG_PROC_SOF_W_PARAM);
    return;
  }

  /* Its time to process Queue */
  queue_length1 = pipeline1->super_param_queue->length;
  queue_length2 = pipeline2->super_param_queue->length;
  min_queue_length = MIN(queue_length1, queue_length2);

  CLOGL(CAM_MCT_MODULE, "queue_length1 [%d] queue_length2 [%d] min_queue_length [%d]",
    queue_length1, queue_length2, min_queue_length);

  mct_queue_traverse(pipeline1->super_param_queue, supermct_print_nodes, NULL);
  mct_queue_traverse(pipeline2->super_param_queue, supermct_print_nodes, NULL);

  for (idx = 1; idx <= min_queue_length; idx++) {
    superparam1 = (mct_event_super_control_parm_t *)
      mct_queue_modify_node_by_idx(pipeline1->super_param_queue, idx);

    superparam2 = (mct_event_super_control_parm_t *)
      mct_queue_modify_node_by_idx(pipeline2->super_param_queue, idx);

    if (superparam1 == NULL || superparam2 == NULL) {
      CLOGL(CAM_MCT_MODULE, "Superparam is null, send without param");
      supermct_wakeup_mct(current_pipeline->bus, sof_msg,
        MCT_BUS_INTMSG_PROC_SOF_WO_PARAM);
      return;
    }

    if ((superparam1->frame_number == superparam2->frame_number) &&
       ((superparam1->is_sync_param) && (superparam2->is_sync_param))) {
      superparam1->applying_frame_id = isp_sof_bus_msg->frame_id;
      superparam2->applying_frame_id = isp_sof_bus_msg->frame_id;
      CLOGL(CAM_MCT_MODULE, "applying_frame_id1 [%d] for frame_number [%d]",
        superparam1->applying_frame_id, superparam1->frame_number);
    } else {
      CLOGL(CAM_MCT_MODULE, "Validation failed: [%d] [%d]",
        superparam1->frame_number, superparam2->frame_number);
      if(superparam1->frame_number < superparam2->frame_number) {
        superparam1->is_sync_param = 0;
      } else {
        superparam2->is_sync_param = 0;
      }
      break;
    }
  }
  supermct_wakeup_mct(current_pipeline->bus, sof_msg,
    MCT_BUS_INTMSG_PROC_SOF_W_PARAM);
}

/** supermct_process_sof:
 *    This function processes the sof message in supermct context
 *    This function calls validate function.
      After validation, it will notify MCT SOF thread with or without parameter.
 *
 *    @super_mct: It takes super_mct object as input
 *    @sof_bus_msg: SOF bus message sent by ISP
 *
 *  Return: VOID
 **/
static boolean supermct_process_sof(supermct_t *l_supermct,
  mct_bus_msg_t *sof_bus_msg)
{
  mct_bus_msg_t *msg = sof_bus_msg;

  if (!SUPERMCT_CHILDREN(l_supermct)) {
    CLOGL(CAM_MCT_MODULE, "SuperMCT do not have child MCT");
    return FALSE;
  }

  if (SUPERMCT_NUM_CHILDREN(l_supermct) > 1) {
    mct_list_operate_nodes_multiple_data(MCT_OBJECT_CHILDREN(l_supermct),
      supermct_validate_params, l_supermct, sof_bus_msg);
  }
  return TRUE;
}

/** supermct_thread:
 *    This thread,
 *        Wait for SOF bus message from ISP
          Processes SOFs
          Validates parameters
          Notify MCT SOF thread
 *    @data: Data is supermct pointer.
 *
 *  Return: VOID *
 **/
static void* supermct_thread(void *data)
{
  mct_bus_msg_t     *sof_bus_msg = NULL;
  uint32_t expected_frame_id = 0;

  supermct_t *l_supermct = (supermct_t *)data;

  CLOGH(CAM_MCT_MODULE, "Creating supermct thread");

  if (l_supermct == NULL) {
    CLOGE(CAM_MCT_MODULE, "supermct is null");
    return NULL;
  }

  do {
    pthread_mutex_lock(&l_supermct->supermct_mutex);
    if (!l_supermct->sof_queue->length) {
      pthread_cond_wait(&l_supermct->supermct_cond,
          &l_supermct->supermct_mutex);
    }
    pthread_mutex_unlock(&l_supermct->supermct_mutex);

    if (l_supermct->sof_queue->length > 5)
      CLOGL(CAM_MCT_MODULE, "Queue length=%d", l_supermct->sof_queue->length);

    while (1) {
      pthread_mutex_lock(&l_supermct->sof_q_mutex);
      sof_bus_msg = (mct_bus_msg_t *)mct_queue_pop_head(l_supermct->sof_queue);
      pthread_mutex_unlock(&l_supermct->sof_q_mutex);

      if (!sof_bus_msg) {
        break;
      }
      if (sof_bus_msg->type == MCT_BUS_MSG_CLOSE_CAM) {
        goto terminate_thread;
      }
      supermct_process_sof(l_supermct, sof_bus_msg);
    }
  } while(1);

terminate_thread:
  CLOGH(CAM_MCT_MODULE, "Terminating supermct thread");
  if (sof_bus_msg->msg) {
    free(sof_bus_msg->msg);
    sof_bus_msg->msg = NULL;
  }
  if (sof_bus_msg) {
    free(sof_bus_msg);
    sof_bus_msg = NULL;
  }
  supermct_remove(l_supermct);
  supermct_deinit(l_supermct);
  CLOGH(CAM_MCT_MODULE, "Terminated supermct thread");
  return NULL;
}

/**
 *      External functions
 *
**/

/** find_supermct_for_pipeline:
 *    It takes pipeline as input and find out parent MCT,
 *    and parent MCT has super MCT or not.
 *
 *    @pipeline: Pipeline as Input to findout supermct exist or not
 *
 *  Return: TRUE, if supermct exist
           FALSE, if supermct do not exist
 **/
supermct_t *find_supermct_for_pipeline(mct_pipeline_t *pipeline)
{
  mct_controller_t *mct = NULL;
  supermct_t *l_supermct = NULL;

  if (!pipeline) {
    CLOGE(CAM_MCT_MODULE, "Invalid pipeline pointer", pipeline);
    return NULL;
  }

  if (MCT_PIPELINE_PARENT(pipeline->object)) {
    mct = MCT_CAST((MCT_PIPELINE_PARENT(pipeline->object))->data);

    if (!mct) {
      CLOGE(CAM_MCT_MODULE, "mct does not exist");
      return NULL;
    }
  } else {
    CLOGD(CAM_MCT_MODULE, "pipeline parent object do not have mct object");
    return NULL;
  }

  if (MCT_PARENT(mct)) {
    l_supermct = SUPERMCT_CAST((MCT_PARENT(mct))->data);

    if (!l_supermct) {
      CLOGE(CAM_MCT_MODULE, "Not valid supermct");
      return NULL;
    }
  } else {
    CLOGD(CAM_MCT_MODULE, "MCT parent object do not have SuperMCT object");
    return NULL;
  }

  return g_supermct;
}

/** supermct_new:
 *    This function create object for supermct
 *
 *    @pipeline: Passing pipeline. need to find parent of pipeline
 *
 *  Return: TRUE, if sucessful
 *         FALSE, if error
 **/
boolean supermct_new(mct_pipeline_t *pipeline)
{
  boolean ret = TRUE;

  /* Initialize supermct */
  if (g_supermct == NULL) {
    if (supermct_init() == FALSE) {
      CLOGE(CAM_MCT_MODULE, "failed");
      return FALSE;
    }
  }

  /* Add parent-child relationship for supermct */
  supermct_append(pipeline, g_supermct);

  CLOGL(CAM_MCT_MODULE,"Number of SuperMCT child %d\n",
    SUPERMCT_NUM_CHILDREN(g_supermct));

  /* Spawn new thread for SuperMCT */
  if (SUPERMCT_NUM_CHILDREN(g_supermct) >= 2) {
    ret = mct_spawn_thread(supermct_thread, g_supermct,
      "CAM_SuperMCT", PTHREAD_CREATE_DETACHED);
  }
  return ret;
}

/** supermct_destroy:
 *    This function destroy supermct
 *
 *    @pipeline: Passing pipeline. need to find parent of pipeline
 *
 *  Return: TRUE, if sucessful
 *         FALSE, if error
 **/
void supermct_destroy(mct_pipeline_t *pipeline)
{
  supermct_t *temp_supermct;
  mct_bus_msg_t *bus_msg_destroy;

  temp_supermct = find_supermct_for_pipeline(pipeline);
  if (!temp_supermct) {
    CLOGL(CAM_MCT_MODULE, "Supermct does not exist for this pipeline");
    return;
  }

  bus_msg_destroy = (mct_bus_msg_t*)malloc(sizeof(mct_bus_msg_t));
  if(!bus_msg_destroy) {
    CLOGL(CAM_MCT_MODULE, "malloc failed");
    return;
  }
  memset(bus_msg_destroy, 0, sizeof(mct_bus_msg_t));
  bus_msg_destroy->type = MCT_BUS_MSG_CLOSE_CAM;

  CLOGL(CAM_MCT_MODULE, "Destroy Supermct");
  pthread_mutex_lock(&temp_supermct->sof_q_mutex);
  mct_queue_push_tail(temp_supermct->sof_queue, bus_msg_destroy);
  pthread_mutex_unlock(&temp_supermct->sof_q_mutex);

  pthread_mutex_lock(&temp_supermct->supermct_mutex);
  pthread_cond_signal(&temp_supermct->supermct_cond);
  pthread_mutex_unlock(&temp_supermct->supermct_mutex);
}

/** is_supermct_exist:
 *    It takes pipeline as input and find out parent MCT,
 *    and parent MCT has super MCT or not.
 *
 *    @pipeline: Pipeline as Input to findout supermct exist or not
 *
 *  Return: TRUE, if supermct exist
           FALSE, if supermct do not exist
 **/
boolean is_supermct_exist(mct_pipeline_t *pipeline)
{
  mct_controller_t *mct = NULL;
  supermct_t *l_supermct = NULL;

  if (!pipeline) {
    CLOGE(CAM_MCT_MODULE, "Invalid pipeline pointer", pipeline);
    return FALSE;
  }

  if (MCT_PIPELINE_PARENT(pipeline->object)) {
    mct = MCT_CAST((MCT_PIPELINE_PARENT(pipeline->object))->data);

    if (!mct) {
      CLOGH(CAM_MCT_MODULE, "mct does not exist in pipeline parent object");
      return FALSE;
    }
  } else {
    CLOGD(CAM_MCT_MODULE, "pipeline parent object do not have mct object");
    return FALSE;
  }

  if (MCT_PARENT(mct)) {
    l_supermct = SUPERMCT_CAST((MCT_PARENT(mct))->data);

    if (!l_supermct) {
      CLOGE(CAM_MCT_MODULE, "Not valid supermct");
      return FALSE;
    }
  } else {
    CLOGD(CAM_MCT_MODULE, "MCT parent object do not have SuperMCT object");
    return FALSE;
  }

  CLOGL(CAM_MCT_MODULE, "supermct exist");
  return TRUE;
}

/** is_dual_zone:
 *    It takes pipeline as input and find out parent MCT,
 *    and parent pipelines are awake or not.
 *
 *    @pipeline: Pipeline as Input to find out Dual camera zone
 *    Dual Camera Zones:
 *           Wide:   Primary: Awake,  Auxiliary: Asleep
 *           Dual:   Primary: Awake,  Auxiliary: Awake
 *           Tele:   Primary: Asleep, Auxiliary: Awake
 *
 *  Return: TRUE, if dual zone
           FALSE, if wide/tele zone
 **/
boolean is_dual_zone(mct_pipeline_t *pipeline)
{
  mct_controller_t *mct = NULL;
  supermct_t *l_supermct = NULL;

  if (!pipeline) {
    CLOGE(CAM_MCT_MODULE, "Invalid pipeline pointer", pipeline);
    return FALSE;
  }

  if (MCT_PIPELINE_PARENT(pipeline->object)) {
    mct = MCT_CAST((MCT_PIPELINE_PARENT(pipeline->object))->data);

    if (!mct) {
      CLOGE(CAM_MCT_MODULE, "supermct does not exist");
      return FALSE;
    }
  } else {
    return FALSE;
  }

  if (MCT_PARENT(mct)) {
    l_supermct = SUPERMCT_CAST((MCT_PARENT(mct))->data);

    if (!l_supermct) {
      CLOGE(CAM_MCT_MODULE, "Not valid supermct");
      return FALSE;
    }
    if (SUPERMCT_NUM_CHILDREN(l_supermct) > 1) {
      mct_list_operate_nodes(MCT_OBJECT_CHILDREN(l_supermct),
        supermct_update_zone, l_supermct);
    }
  } else {
    return FALSE;
  }

  return ((SUPERMCT_DUAL_ZONE == l_supermct->supermct_zone) ?
      TRUE : FALSE);
}

/** supermct_notify:
 *    This function notify/wakeup supermct thread about SOF message
 *
 *    @bus_msg: bus message is input. It is receives from ISP.
 *
 *  Return: TRUE, sucessful
           FALSE, Failure
 **/
boolean supermct_notify(mct_bus_msg_t *bus_msg)
{
  mct_bus_msg_t *local_msg = NULL;
  mct_bus_msg_isp_sof_t *isp_sof_bus_msg = NULL;

  if (!bus_msg) {
    CLOGE(CAM_MCT_MODULE, "bus msg is null");
    return FALSE;
  }

  if (bus_msg->type >= MCT_BUS_MSG_MAX) {
    CLOGE(CAM_MCT_MODULE, "bus_msg type %d is not valid", bus_msg->type);
    return FALSE;
  }

  if (!g_supermct) {
    CLOGL(CAM_MCT_MODULE, "supermct is null");
    return FALSE;
  }

  local_msg = malloc(sizeof(mct_bus_msg_t));
  if (!local_msg) {
    CLOGE(CAM_MCT_MODULE, "Can't allocate memory");
    return FALSE;
  }

  local_msg->sessionid = bus_msg->sessionid;
  local_msg->type = bus_msg->type;
  local_msg->size = bus_msg->size;

  if (local_msg->size) {
    local_msg->msg = malloc(local_msg->size);
    if (!local_msg->msg) {
      CLOGE(CAM_MCT_MODULE, "Can't allocate memory");
      free(local_msg);
      return FALSE;
    }
    isp_sof_bus_msg = (mct_bus_msg_isp_sof_t *)local_msg->msg;
    memcpy(local_msg->msg, bus_msg->msg, local_msg->size);
    CLOGL(CAM_MCT_MODULE, "frame_id=%d", isp_sof_bus_msg->frame_id);
  } else {
    local_msg->msg = NULL;
  }

  pthread_mutex_lock(&(g_supermct->sof_q_mutex));
  mct_queue_push_tail(g_supermct->sof_queue, local_msg);
  pthread_mutex_unlock(&g_supermct->sof_q_mutex);

  if (g_supermct->sof_queue->length > 5)
    CLOGL(CAM_MCT_MODULE, "Queue length=%d", g_supermct->sof_queue->length);

  pthread_mutex_lock(&g_supermct->supermct_mutex);
  pthread_cond_signal(&g_supermct->supermct_cond);
  pthread_mutex_unlock(&g_supermct->supermct_mutex);
  return TRUE;
}

/** supermct_update_ack_id:
 *    This function keep track of last sof request has been made.
 *    It takes sof id as input.
 *
 *    @pipeline: it takes pipeline as input.
 *
 *  Return: VOID
 **/
void supermct_update_ack_id(unsigned int sof_id)
{
  g_supermct->last_valid_sof_id = sof_id;
}

/** supermct_lock_sof_update:
 *    This function is to block sof update in supermct.
 *
 *  Return: VOID
 **/
void supermct_lock_sof_update()
{
    pthread_mutex_lock(&g_supermct->update_sof_id_mutex);
}

/** supermct_unlock_sof_update:
 *    This function is to unblock sof update in supermct.
 *
 *  Return: VOID
 **/
void supermct_unlock_sof_update()
{
    pthread_mutex_unlock(&g_supermct->update_sof_id_mutex);
}
