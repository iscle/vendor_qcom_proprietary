/**
  \file **************************************************************************
  *
  *  A U D I O   C A L I B R A T I O N   P A C K E T   H A N D L E R
  *
  *DESCRIPTION
  * This file contains the implementation of online_intf
  *
  *REFERENCES
  * None.
  *
  * Copyright (c) 2011-2017 by Qualcomm Technologies, Inc.
  * All Rights Reserved.
  * Confidential and Proprietary - Qualcomm Technologies, Inc.
  *******************************************************************************
  */
/**
  \file ***************************************************************************
  *
  *                      EDIT HISTORY FOR FILE
  *
  *  This section contains comments describing changes made to this file.
  *  Notice that changes are listed in reverse chronological order.
  *
  *  $Header: acph.h
  *
  *when         who     what, where, why
  *--------     ---     ----------------------------------------------------------
  *06/09/16  mahadevk Suppress warning for unused parameters
  *08/05/15  mahadevk Updated Batch set_data operation command
  *07/29/15  mahadevk Support commands to perform Batch set_data operation
  *05/28/14     mh      SW migration from 32-bit to 64-bit architecture
  *02/14/14     avi     Support commands for ACDB persistence.
  *06/07/13     avi     Support Voice Volume boost feature
  *08/03/11     ernanl  initial draft
  ********************************************************************************
  */
/* $Header: //source/qcom/qct/multimedia2/Audio/audcal4/acdb_sw/dev/MDF/acph_online/src/acph_online.c#2 $ */
/*
   -------------------------------
   |Include Files                |
   -------------------------------
   */

#include "acdb_os_includes.h"
#include "acph_online.h"
#include "acdb_command.h"
#include "acdb_datainfo.h"
#include "acdb_utility.h"
/*===========================================================================
  Macro
  ===========================================================================*/

const uint32_t ACDB_TARGET_VERSION = 0x00012A7B;

/*===========================================================================
  External VARIABLES
  ===========================================================================*/
//extern char_t * acph_main_buffer;

/*===========================================================================
  Internal VARIABLE
  ===========================================================================*/

/**
 * FUNCTION : get_target_version
 *
 * DESCRIPTION : Get target version from ACDB
 *
 * DEPENDENCIES : ACDB needs to be available and initialized
 *
 * PARAMS:
 *   req_buf_ptr - pointer to request buffer
 *   resp_buf_ptr - pointer to response buffer
 *   resp_buf_length - length of the response buffer
 *
 * RETURN VALUE : None
 *
 * SIDE EFFECTS : None
 */
static int32_t get_target_version(uint8_t *req_buf_ptr,
   uint32_t req_buf_len,
   uint8_t *resp_buf_ptr,
   uint32_t resp_buf_length,
   uint32_t *resp_buf_bytes_filled
   )
{
   UNUSED(*req_buf_ptr);
   UNUSED(req_buf_len);
   if ((NULL == resp_buf_ptr) || (resp_buf_length < sizeof(uint32_t)))
   {
      return ACPH_ERR_OUT_OF_BUFFER_SIZE;
   }
   else
   {
      ACDB_MEM_CPY((void*)(resp_buf_ptr), sizeof(uint32_t), (void*)&ACDB_TARGET_VERSION, sizeof(uint32_t));
      *resp_buf_bytes_filled = sizeof(uint32_t);
      return ACPH_SUCCESS;
   }
}

/**
 * FUNCTION : check_connection
 *
 * DESCRIPTION : check connection
 *
 * DEPENDENCIES : none
 *
 * PARAMS:
 *   req_buf_ptr - pointer to request buffer
 *   resp_buf_ptr - pointer to response buffer
 *   resp_buf_length - length of the response buffer
 *
 * RETURN VALUE : None
 *
 * SIDE EFFECTS : None
 */
static int32_t check_connection(uint8_t *req_buf_ptr,
   uint32_t req_buf_len,
   uint8_t *resp_buf_ptr,
   uint32_t resp_buf_length,
   uint32_t *resp_buf_bytes_filled
   )
{
   UNUSED(*req_buf_ptr);
   UNUSED(req_buf_len);
   UNUSED(*resp_buf_ptr);
   UNUSED(resp_buf_length);
   UNUSED(*resp_buf_bytes_filled);
   return ACPH_SUCCESS;
}

/**
 * FUNCTION : query_max_buffer_length
 *
 * DESCRIPTION : get related maximum buffer length
 *
 * DEPENDENCIES : ACPH needs to be initialized
 *
 * PARAMS:
 *   req_buf_ptr - pointer to request buffer
 *   resp_buf_ptr - pointer to response buffer
 *   resp_buf_length - length of the response buffer
 *
 * INPUT: None
 *
 * RETURN VALUE :
 *      32-bit ACPH BUFFER SIZE
 *
 * SIDE EFFECTS : None
 */
static int32_t query_max_buffer_length(uint8_t *req_buf_ptr,
   uint32_t req_buf_len,
   uint8_t *resp_buf_ptr,
   uint32_t resp_buf_length,
   uint32_t *resp_buf_bytes_filled
   )
{
   uint32_t acph_buf_len = resp_buf_length;

   UNUSED(req_buf_len);
   UNUSED(*req_buf_ptr);
   if ((NULL == resp_buf_ptr) || (resp_buf_length < sizeof(uint32_t)))
   {
      return ACPH_ERR_OUT_OF_BUFFER_SIZE;
   }
   else
   {
      ACDB_MEM_CPY((void*)(resp_buf_ptr), sizeof(uint32_t), (void*)&acph_buf_len, sizeof(uint32_t));
      *resp_buf_bytes_filled = sizeof(uint32_t);
      return ACPH_SUCCESS;
   }
}

/**
 * FUNCTION : get_acdb_files_info
 *
 * DESCRIPTION : get the files info which were currently loaded in the memory
 *
 * DEPENDENCIES : ACPH needs to be initialized
 *
 * PARAMS:
 *   req_buf_ptr - pointer to request buffer
 *   resp_buf_ptr - pointer to response buffer
 *   resp_buf_length - length of the response buffer
 *
 * INPUT: None
 *
 * RETURN VALUE :
 *      32-bit ACPH BUFFER SIZE
 *
 * SIDE EFFECTS : None
 */
static int32_t get_acdb_files_info(uint8_t *req_buf_ptr,
   uint32_t req_buf_len,
   uint8_t *resp_buf_ptr,
   uint32_t resp_buf_length,
   uint32_t *resp_buf_bytes_filled
   )
{
   //uint32_t acph_buf_len = resp_buf_length;
   AcdbQueryCmdType cmd;
   AcdbQueryResponseType rsp;
   int32_t result = ACPH_SUCCESS;
   UNUSED(req_buf_len);
   UNUSED(*req_buf_ptr);

   if (NULL == resp_buf_ptr)
   {
      /**not initilized*/
      return ACPH_ERR_OUT_OF_BUFFER_SIZE;
   }

   cmd.nBufferLength = resp_buf_length;
   cmd.pBufferPointer = resp_buf_ptr;
   result = AcdbCmdGetFilesInfo(&cmd, &rsp);
   if (result == ACPH_SUCCESS)
      *resp_buf_bytes_filled = rsp.nBytesUsedInBuffer;
   return result;
}

/**
 * FUNCTION : get_no_of_tbl_entries_on_heap
 *
 * DESCRIPTION : get the heap info which were currently loaded in the memory
 *
 * DEPENDENCIES : ACPH needs to be initialized
 *
 * PARAMS:
 *   req_buf_ptr - pointer to request buffer
 *   resp_buf_ptr - pointer to response buffer
 *   resp_buf_length - length of the response buffer
 *
 * INPUT: None
 *
 * RETURN VALUE :
 *      32-bit ACPH BUFFER SIZE
 *
 * SIDE EFFECTS : None
 */
static int32_t get_no_of_tbl_entries_on_heap(uint8_t *req_buf_ptr,
   uint32_t req_buf_len,
   uint8_t *resp_buf_ptr,
   uint32_t resp_buf_length,
   uint32_t *resp_buf_bytes_filled
   )
{
   //uint32_t acph_buf_len = ACPH_BUFFER_LENGTH - ACPH_ACDB_BUFFER_POSITION;
   int32_t result = ACPH_SUCCESS;

   AcdbQueryNoOfTblEntriesCmdType cmd;
   AcdbRespNoOfTblEntriesCmdType rsp = { 0 };
   if (NULL == resp_buf_ptr)
   {
      /**not initilized*/
      return ACPH_ERR_OUT_OF_BUFFER_SIZE;
   }

   if (req_buf_len != sizeof(AcdbQueryNoOfTblEntriesCmdType))
   {
      /**command parameter missing*/
      ACDB_DEBUG_LOG("Invalid getheapdata request made to target from client.Insufficient data provided to process the req");
      return ACPH_ERR_ACDB_COMMAND_FAILURE;
   }
   if (resp_buf_length < sizeof(rsp.nNoOfEntries))
   {
      return ACPH_ERR_OUT_OF_BUFFER_SIZE;
   }
   ACDB_MEM_CPY((void *)&cmd.nTblId, sizeof(cmd.nTblId), req_buf_ptr, sizeof(cmd.nTblId));

   result = AcdbCmdGetNoOfTblEntriesOnHeap((uint8_t*)&cmd, sizeof(cmd), (uint8_t*)&rsp, sizeof(rsp));
   if (result == ACPH_SUCCESS)
   {
      ACDB_MEM_CPY((resp_buf_ptr), sizeof(rsp.nNoOfEntries), &rsp.nNoOfEntries, sizeof(rsp.nNoOfEntries));
      *resp_buf_bytes_filled = sizeof(rsp.nNoOfEntries);
   }
   return result;
}

/**
 * FUNCTION : get_tbl_entries_on_heap
 *
 * DESCRIPTION : get the heap info which were currently loaded in the memory
 *
 * DEPENDENCIES : ACPH needs to be initialized
 *
 * PARAMS:
 *   req_buf_ptr - pointer to request buffer
 *   resp_buf_ptr - pointer to response buffer
 *   resp_buf_length - length of the response buffer
 *
 * INPUT: None
 *
 * RETURN VALUE :
 *      32-bit ACPH BUFFER SIZE
 *
 * SIDE EFFECTS : None
 */
static int32_t get_tbl_entries_on_heap(uint8_t *req_buf_ptr,
   uint32_t req_buf_len,
   uint8_t *resp_buf_ptr,
   uint32_t resp_buf_length,
   uint32_t *resp_buf_bytes_filled
   )
{
   //uint32_t acph_buf_len = ACPH_BUFFER_LENGTH - ACPH_ACDB_BUFFER_POSITION;
   int32_t result = ACPH_SUCCESS;

   uint32_t offset = 0;

   AcdbQueryTblEntriesCmdType cmd;
   AcdbQueryResponseType rsp = { 0 };
   if (NULL == resp_buf_ptr)
   {
      /**not initilized*/
      return ACPH_ERR_OUT_OF_BUFFER_SIZE;
   }

   if (req_buf_len != (2 * sizeof(uint32_t)))
   {
      /**command parameter missing*/
      ACDB_DEBUG_LOG("Invalid getheapdata request made to target from client.Insufficient data provided to process the req");
      return ACPH_ERR_ACDB_COMMAND_FAILURE;
   }
   ACDB_MEM_CPY((void *)&cmd.nTblId, sizeof(cmd.nTblId), req_buf_ptr + offset, sizeof(cmd.nTblId));
   offset += sizeof(cmd.nTblId);

   ACDB_MEM_CPY((void *)&cmd.nTblEntriesOffset, sizeof(cmd.nTblEntriesOffset), req_buf_ptr + offset, sizeof(cmd.nTblEntriesOffset));
   offset += sizeof(cmd.nTblEntriesOffset);

   //memcpy((void *)&cmd.nRequiredNoOfTblEntries,req_buf_ptr + offset,sizeof(cmd.nRequiredNoOfTblEntries));
   //offset += sizeof(cmd.nRequiredNoOfTblEntries);

   cmd.pBuff = resp_buf_ptr;
   cmd.nBuffSize = resp_buf_length;

   result = AcdbCmdGetTblEntriesOnHeap((uint8_t*)&cmd, sizeof(cmd), (uint8_t*)&rsp, sizeof(rsp));
   if (result == ACPH_SUCCESS)
   {
      *resp_buf_bytes_filled = rsp.nBytesUsedInBuffer;
   }
   return result;
}

/**
 * FUNCTION : get_acdb_file
 *
 * DESCRIPTION : gets the acdb file which was requested by client
 *
 * DEPENDENCIES : ACPH needs to be initialized
 *
 * PARAMS:
 *   req_buf_ptr - pointer to request buffer
 *   resp_buf_ptr - pointer to response buffer
 *   resp_buf_length - length of the response buffer
 *
 * INPUT: None
 *
 * RETURN VALUE :
 *      32-bit ACPH BUFFER SIZE
 *
 * SIDE EFFECTS : None
 */
static int32_t get_acdb_file(uint8_t *req_buf_ptr,
   uint32_t req_buf_len,
   uint8_t *resp_buf_ptr,
   uint32_t resp_buf_length,
   uint32_t *resp_buf_bytes_filled
   )
{
   //uint32_t acph_buf_len = ACPH_BUFFER_LENGTH - ACPH_ACDB_BUFFER_POSITION;
   AcdbCmdGetFileDataReq req;
   AcdbCmdResp resp;
   uint32_t offset = 0;
   int32_t result = ACPH_SUCCESS;
   UNUSED(req_buf_len);
   if (NULL == resp_buf_ptr)
   {
      /**not initilized*/
      return ACPH_ERR_OUT_OF_BUFFER_SIZE;
   }
   ACDB_MEM_CPY(&req.nfile_offset, sizeof(req.nfile_offset), req_buf_ptr + offset, sizeof(req.nfile_offset));
   offset += sizeof(req.nfile_offset);
   ACDB_MEM_CPY(&req.nfile_data_len, sizeof(req.nfile_data_len), req_buf_ptr + offset, sizeof(req.nfile_data_len));
   offset += sizeof(req.nfile_data_len);
   ACDB_MEM_CPY(&req.nfileNameLen, sizeof(req.nfileNameLen), req_buf_ptr + offset, sizeof(req.nfileNameLen));
   offset += sizeof(req.nfileNameLen);
   req.pFileName = req_buf_ptr + offset;

   resp.pRespBuff = resp_buf_ptr;
   resp.nresp_buff_len = resp_buf_length;

   result = AcdbCmdGetFileData(&req, &resp);
   if (result == ACPH_SUCCESS)
   {
      *resp_buf_bytes_filled = resp.nresp_buff_filled;
   }
   return result;
}

/**
 * FUNCTION : get_acdb_data
 *
 * DESCRIPTION : gets the acdb data which was requested by client
 *
 * DEPENDENCIES : ACPH needs to be initialized
 *
 * PARAMS:
 *   req_buf_ptr - pointer to request buffer
 *   resp_buf_ptr - pointer to response buffer
 *   resp_buf_length - length of the response buffer
 *
 * INPUT: None
 *
 * RETURN VALUE :
 *      32-bit ACPH BUFFER SIZE
 *
 * SIDE EFFECTS : None
 */
static int32_t get_acdb_data(uint8_t *req_buf_ptr,
   uint32_t req_buf_len,
   uint8_t *resp_buf_ptr,
   uint32_t resp_buf_length,
   uint32_t *resp_buf_bytes_filled
   )
{
   //uint32_t acph_buf_len = ACPH_BUFFER_LENGTH - ACPH_ACDB_BUFFER_POSITION;
   uint32_t offset = 0;
   //uint32_t reqdatalen = 0;
   int32_t result = ACPH_SUCCESS;
   int32_t tblId = 0;
   int32_t mid = 0;
   int32_t pid = 0;
   int32_t iid = 0;
   uint8_t *pIndices = NULL;
   uint32_t nBytesFilled = 0;
   uint32_t noOfTableIndices = 0;
   uint32_t nonModuleTblFound = 0;
   uint32_t noOfCdftIndices = 0;
   uint32_t noOfCmdIndices = 0;
   uint8_t *pRspBuff = NULL;
   uint32_t expectedBufSize = 0;

   if (NULL == resp_buf_ptr)
   {
      /**not initilized*/
      return ACPH_ERR_UNKNOWN_REASON;
   }
   //memcpy((void *)&reqdatalen,req_buf_ptr + ACPH_DATA_LENGTH_POSITION,ACPH_DATA_LENGTH_LENGTH);
   if ((req_buf_len == 0) || ((req_buf_len % 4) != 0))
   {
      /**command parameter missing*/
      ACDB_DEBUG_LOG("Invalid getdata request made to target from client.Insufficient data provided to process the req");
      return ACPH_ERR_ACDB_COMMAND_FAILURE;
   }
   ACDB_MEM_CPY((void *)&tblId, sizeof(tblId), req_buf_ptr + offset, sizeof(tblId));
   offset += sizeof(uint32_t);

   result = Get_table_indices_count(tblId, &noOfTableIndices, &nonModuleTblFound, &noOfCdftIndices,&noOfCmdIndices);
   if (result != ACDB_SUCCESS)
   {
      ACDB_DEBUG_LOG("[ACPH Online]->[set_acdb_data]->Failed. Could not find number of indices of the table:[%u]\n", tblId);
      result = ACPH_ERR_UNKNOWN_REASON;
      return result;
   }

   if (noOfCdftIndices == INSTANCE_CDFT_INDICES_COUNT)
   {
      expectedBufSize = (sizeof(tblId) + (noOfTableIndices*sizeof(uint32_t)) + sizeof(mid) + sizeof(iid) + sizeof(pid));
   }
   else
   {
      expectedBufSize = (sizeof(tblId) + (noOfTableIndices*sizeof(uint32_t)) + sizeof(mid) + sizeof(pid));
   }

   if ((nonModuleTblFound == 0) &&
      (req_buf_len != expectedBufSize))
   {
      ACDB_DEBUG_LOG("Invalid getdata request made to target from client.Provided insufficient no of table params");
      return ACPH_ERR_ACDB_COMMAND_FAILURE;
   }
   else if ((nonModuleTblFound == 1) &&
      (req_buf_len != (sizeof(tblId) + (noOfTableIndices*sizeof(uint32_t)))))
   {
      ACDB_DEBUG_LOG("Invalid getdata request made to target from client.Provided insufficient no of table params");
      return ACPH_ERR_ACDB_COMMAND_FAILURE;
   }

   pIndices = (uint8_t *)(req_buf_ptr + offset);
   offset += (noOfTableIndices*sizeof(uint32_t));
   if (nonModuleTblFound == 0)
   {
      ACDB_MEM_CPY((void *)&mid, sizeof(uint32_t), (req_buf_ptr + offset), sizeof(uint32_t));
      offset += sizeof(uint32_t);
      if (noOfCdftIndices == INSTANCE_CDFT_INDICES_COUNT)
      {
         ACDB_MEM_CPY((void *)&iid, sizeof(uint32_t), (req_buf_ptr + offset), sizeof(uint32_t));
         offset += sizeof(uint32_t);
      }
      ACDB_MEM_CPY((void *)&pid, sizeof(uint32_t), (req_buf_ptr + offset), sizeof(uint32_t));
   }

   pRspBuff = resp_buf_ptr;

   result = AcdbCmdGetOnlineDataV2(tblId, pIndices, noOfTableIndices, noOfCdftIndices, pRspBuff, resp_buf_length, &nBytesFilled);
   if (result == ACPH_SUCCESS)
   {
      *resp_buf_bytes_filled = nBytesFilled;
   }
   return result;
}

/**
 * FUNCTION : set_acdb_data
 *
 * DESCRIPTION : set the acdb file which was requested by client
 *
 * DEPENDENCIES : ACPH needs to be initialized
 *
 * PARAMS:
 *   req_buf_ptr - pointer to request buffer
 *   resp_buf_ptr - pointer to response buffer
 *   resp_buf_length - length of the response buffer
 *
 * INPUT: None
 *
 * RETURN VALUE :
 *      32-bit ACPH BUFFER SIZE
 *
 * SIDE EFFECTS : None
 */
static int32_t set_acdb_data(uint8_t *req_buf_ptr,
   uint32_t req_buf_len,
   uint8_t *resp_buf_ptr,
   uint32_t resp_buf_length,
   uint32_t *resp_buf_bytes_filled
   )
{
   uint32_t offset = 0;
   int32_t result = ACPH_SUCCESS;
   uint32_t tblId = 0;
   int32_t mid = 0;
   int32_t pid = 0;
   int32_t iid = 0;
   uint8_t *pIndices = NULL;
   uint32_t noOfTableIndices = 0;
   uint32_t nonModuleTblFound = 0;
   uint32_t noOfCdftIndices = 0;
   uint32_t noOfCmdIndices = 0;
   uint8_t *pInBuff = NULL;
   uint32_t nInDataBufLen = 0;
	uint32_t persistData = FALSE;
   uint32_t persistanceSupported = FALSE;
   int32_t persistResult = ACDB_ERROR;
   uint32_t expectedBufSize = 0;
   uint32_t cdftIndices = 0;
   uint8_t *buffer = NULL;

   UNUSED(*resp_buf_bytes_filled);
   UNUSED(resp_buf_length);
   UNUSED(*resp_buf_ptr);

   //memcpy((void *)&reqdatalen,req_buf_ptr + ACPH_DATA_LENGTH_POSITION,ACPH_DATA_LENGTH_LENGTH);
   if (req_buf_len == 0)
   {
      /**command parameter missing*/
      ACDB_DEBUG_LOG("Invalid getdata request made to target from client.Insufficient data provided to process the req");
      return ACPH_ERR_ACDB_COMMAND_FAILURE;
   }
   else if ((req_buf_len % 4) != 0)
   {
      ACDB_DEBUG_LOG("ACPH:Warning The set data request provided is not 4 byte aligned");
   }
   ACDB_MEM_CPY((void *)&tblId, sizeof(tblId), req_buf_ptr + offset, sizeof(tblId));
   offset += sizeof(uint32_t);
   result = Get_table_indices_count(tblId, &noOfTableIndices, &nonModuleTblFound, &noOfCdftIndices,&noOfCmdIndices);
   if (result != ACDB_SUCCESS)
   {
      ACDB_DEBUG_LOG("[ACPH Online]->[set_acdb_data]->Failed. Could not find number of indices of the table:[%u]\n", tblId);
      result = ACPH_ERR_UNKNOWN_REASON;
      return result;
   }

   if (noOfCdftIndices == INSTANCE_CDFT_INDICES_COUNT)
   {
      expectedBufSize = (sizeof(tblId) + (noOfTableIndices*sizeof(uint32_t)) + sizeof(mid) + sizeof(iid) + sizeof(pid));
   }
   else
   {
      expectedBufSize = (sizeof(tblId) + (noOfTableIndices*sizeof(uint32_t)) + sizeof(mid) + sizeof(pid));
   }

   if ((nonModuleTblFound == 0) &&
      (req_buf_len <= expectedBufSize))
   {
      ACDB_DEBUG_LOG("Invalid getdata request made to target from client.Provided insufficient no of table params");
      return ACPH_ERR_ACDB_COMMAND_FAILURE;
   }
   else if ((nonModuleTblFound == 1) &&
      (req_buf_len <= (sizeof(tblId) + (noOfTableIndices*sizeof(uint32_t)))))
   {
      ACDB_DEBUG_LOG("Invalid getdata request made to target from client.Provided insufficient no of table params");
      return ACPH_ERR_ACDB_COMMAND_FAILURE;
   }

   pIndices = (uint8_t *)(req_buf_ptr + offset);
   offset += (noOfTableIndices*sizeof(uint32_t));
   if (nonModuleTblFound == 0)
   {
      ACDB_MEM_CPY((void *)&mid, sizeof(uint32_t), (req_buf_ptr + offset), sizeof(uint32_t));
      offset += sizeof(uint32_t);
      if (noOfCdftIndices == INSTANCE_CDFT_INDICES_COUNT)
      {
         ACDB_MEM_CPY((void *)&iid, sizeof(uint32_t), (req_buf_ptr + offset), sizeof(uint32_t));
         offset += sizeof(uint32_t);
      }
      ACDB_MEM_CPY((void *)&pid, sizeof(uint32_t), (req_buf_ptr + offset), sizeof(uint32_t));
      offset += sizeof(uint32_t);
   }

   pInBuff = req_buf_ptr + offset;
   if (nonModuleTblFound == 0)
   {
      if (noOfCdftIndices == INSTANCE_CDFT_INDICES_COUNT)
      {
         nInDataBufLen = req_buf_len - ((noOfTableIndices*sizeof(uint32_t)) + sizeof(tblId) + sizeof(mid) + sizeof(iid) + sizeof(pid));
      }
      else
      {
         nInDataBufLen = req_buf_len - ((noOfTableIndices*sizeof(uint32_t)) + sizeof(tblId) + sizeof(mid) + sizeof(pid));
      }
   }
   else
   {
      nInDataBufLen = req_buf_len - ((noOfTableIndices*sizeof(uint32_t)) + sizeof(tblId));
   }

   buffer = (uint8_t *)(malloc((noOfTableIndices + noOfCdftIndices)*sizeof(uint32_t)));
   if(buffer == NULL)
   {
	  ACDB_DEBUG_LOG("[ACPH Online]->[set_acdb_data]->Unable to allocate memory for temporary buffer\n");
	  return ACPH_FAILURE;
   }
   ACDB_MEM_CPY(buffer, noOfTableIndices*sizeof(uint32_t), pIndices, noOfTableIndices*sizeof(uint32_t));
   offset = noOfTableIndices*sizeof(uint32_t);
   //Copy MID/IID/PID to pIndices
   ACDB_MEM_CPY(buffer + offset, sizeof(uint32_t), &mid, sizeof(mid));
   offset += sizeof(mid);
   if (noOfCdftIndices == INSTANCE_CDFT_INDICES_COUNT)
   {
      ACDB_MEM_CPY(buffer + offset, sizeof(uint32_t), &iid, sizeof(iid));
      offset += sizeof(iid);
   }
   ACDB_MEM_CPY(buffer + offset, sizeof(uint32_t), &pid, sizeof(pid));
   offset += sizeof(pid);

   pIndices = buffer;
   result = AcdbCmdSetOnlineDataV2(persistData, tblId, pIndices, noOfTableIndices, noOfCdftIndices, pInBuff, nInDataBufLen);
   if (result != 0)
   {
      return result;
   }
   if (persistData == TRUE)
   {
      persistResult = AcdbCmdIsPersistenceSupported(&persistanceSupported);
      if (persistResult == ACDB_SUCCESS)
      {
         if (persistanceSupported == TRUE)
         {
            result = AcdbCmdSaveDeltaFileData();
            if (result != ACDB_SUCCESS)
            {
               ACDB_DEBUG_LOG("[ACPH Online]->[set_acdb_data]->Unable to save delta file data\n");
            }
         }
      }
   }
   return result;
}

/**
* FUNCTION : batch_set_acdb_data
*
* DESCRIPTION : batch set acdb data requested by client
*
* DEPENDENCIES : ACPH needs to be initialized
*
* PARAMS:
*   req_buf_ptr - pointer to request buffer
*   resp_buf_ptr - pointer to response buffer
*   resp_buf_length - length of the response buffer
*
* INPUT: None
*
* RETURN VALUE :
*      32-bit ACPH BUFFER SIZE
*
* SIDE EFFECTS : None
*/
static int32_t batch_set_acdb_data(uint8_t *req_buf_ptr,
   uint32_t req_buf_len,
   uint8_t *resp_buf_ptr,
   uint32_t resp_buf_length,
   uint32_t *resp_buf_bytes_filled
   )
{
   uint32_t offset = 0, tempOffset = 0;
   uint32_t failed_indices_len = 0;
   uint32_t fail_indices_offset = 4;
   int32_t final_res = 0;
   int32_t result = ACPH_SUCCESS;
   uint32_t tblId = 0;
   int32_t mid = 0;
   int32_t iid = 0;
   int32_t pid = 0;
   uint32_t indices_len = 0;
   uint8_t *pIndices = NULL;
   uint32_t noOfTableIndices = 0;
   uint32_t nonModuleTblFound = 0;
   uint32_t noOfCdftIndices = 0;
   uint32_t noOfCmdIndices = 0;
   uint8_t *pInBuff = NULL;
   uint32_t nInDataBufLen = 0;
   uint32_t persistData = FALSE;
   uint32_t persistanceSupported = FALSE;
   int32_t persistResult = ACDB_ERROR;
   uint32_t cnt;
   uint8_t *buffer = NULL;
   UNUSED(resp_buf_length);

   //memcpy((void *)&reqdatalen,req_buf_ptr + ACPH_DATA_LENGTH_POSITION,ACPH_DATA_LENGTH_LENGTH);
   if (req_buf_len == 0)
   {
      /**command parameter missing*/
      ACDB_DEBUG_LOG("Invalid Batch set data request made to target from client.Insufficient data provided to process the req");
      return ACPH_ERR_ACDB_COMMAND_FAILURE;
   }
   else if ((req_buf_len % 4) != 0)
   {
      ACDB_DEBUG_LOG("ACPH:Warning The batch set data request provided is not 4 byte aligned");
   }
   ACDB_MEM_CPY((void *)&tblId, sizeof(tblId), req_buf_ptr + offset, sizeof(tblId));
   offset += sizeof(uint32_t);

   ACDB_MEM_CPY((void *)&nInDataBufLen, sizeof(nInDataBufLen), req_buf_ptr + offset, sizeof(nInDataBufLen));
   offset += sizeof(uint32_t);

   result = Get_table_indices_count(tblId, &noOfTableIndices, &nonModuleTblFound, &noOfCdftIndices,&noOfCmdIndices);
   if (result != ACPH_SUCCESS)
   {
      ACDB_DEBUG_LOG("[ACPH Online]->[batch_set_acdb_data]->Failed. Could not find number of indices of the table:[%u]\n", tblId);
      return result;
   }

   if(noOfCdftIndices == NON_INSTANCE_CDFT_INDICES_COUNT)
   {
	   if ((nonModuleTblFound == 0) &&
		   (req_buf_len < (sizeof(tblId) + sizeof(nInDataBufLen) + (nInDataBufLen * sizeof(uint8_t)) + sizeof(uint32_t) + (noOfTableIndices*sizeof(uint32_t)) + sizeof(mid) + sizeof(pid))))
	   {
		   ACDB_DEBUG_LOG("Invalid batch set data request made to target from client.Provided insufficient no of table params");
		   return ACPH_ERR_ACDB_COMMAND_FAILURE;
	   }
	   else if ((nonModuleTblFound == 1) &&
		   (req_buf_len < (sizeof(tblId) + sizeof(nInDataBufLen) + (nInDataBufLen * sizeof(uint8_t)) + sizeof(uint32_t) + (noOfTableIndices*sizeof(uint32_t)))))
	   {
		   ACDB_DEBUG_LOG("Invalid batch set data request made to target from client.Provided insufficient no of table params");
		   return ACPH_ERR_ACDB_COMMAND_FAILURE;
	   }
   }
   else if(noOfCdftIndices == INSTANCE_CDFT_INDICES_COUNT)
   {
	   if ((nonModuleTblFound == 0) &&
		   (req_buf_len < (sizeof(tblId) + sizeof(nInDataBufLen) + (nInDataBufLen * sizeof(uint8_t)) + sizeof(uint32_t) + (noOfTableIndices*sizeof(uint32_t)) + sizeof(mid) + sizeof(iid) + sizeof(pid))))
	   {
		   ACDB_DEBUG_LOG("Invalid batch set data request made to target from client.Provided insufficient no of table params");
		   return ACPH_ERR_ACDB_COMMAND_FAILURE;
	   }
	   else if ((nonModuleTblFound == 1) &&
		   (req_buf_len < (sizeof(tblId) + sizeof(nInDataBufLen) + (nInDataBufLen * sizeof(uint8_t)) + sizeof(uint32_t) + (noOfTableIndices*sizeof(uint32_t)))))
	   {
		   ACDB_DEBUG_LOG("Invalid batch set data request made to target from client.Provided insufficient no of table params");
		   return ACPH_ERR_ACDB_COMMAND_FAILURE;
	   }
   }

   

   pInBuff = req_buf_ptr + offset;
   offset += (nInDataBufLen*sizeof(uint8_t));

   ACDB_MEM_CPY((void *)&indices_len, sizeof(indices_len), req_buf_ptr + offset, sizeof(indices_len));
   offset += sizeof(uint32_t);
   for (cnt = 0; cnt < indices_len;)
   {
      pIndices = (uint8_t *)(req_buf_ptr + offset);
      offset += ((noOfTableIndices + noOfCdftIndices)*sizeof(uint32_t));
      cnt = cnt + noOfTableIndices + noOfCdftIndices;

      result = AcdbCmdSetOnlineDataV2(persistData, tblId, pIndices, noOfTableIndices, noOfCdftIndices, pInBuff, nInDataBufLen);

      /*if(cnt%2 == 0)
      {
      result = ACDB_ERROR;
      }*/
      if (result != 0)
      {
         ACDB_MEM_CPY((void *)(resp_buf_ptr + fail_indices_offset), noOfTableIndices * sizeof(uint32_t), pIndices, noOfTableIndices * sizeof(uint32_t));
         fail_indices_offset += (noOfTableIndices * sizeof(uint32_t));
         failed_indices_len += noOfTableIndices;
         if (nonModuleTblFound == 0)
         {
            ACDB_MEM_CPY((void *)(resp_buf_ptr + fail_indices_offset), sizeof(mid), &mid, sizeof(mid));
            fail_indices_offset += sizeof(mid);

            ACDB_MEM_CPY((void *)(resp_buf_ptr + fail_indices_offset), sizeof(pid), &pid, sizeof(pid));
            fail_indices_offset += sizeof(pid);
            failed_indices_len += 2;
         }
      }
   }

   if (failed_indices_len == indices_len)
   {
      final_res = -1; //unsuccessful evaluation
   }
   else if (failed_indices_len > 0 && failed_indices_len < indices_len)
   {
      final_res = -2; //partial success
   }
   if (failed_indices_len > 0)
   {
      ACDB_MEM_CPY((void *)resp_buf_ptr, sizeof(uint32_t), &failed_indices_len, sizeof(uint32_t));

      *resp_buf_bytes_filled = failed_indices_len*sizeof(uint32_t) + sizeof(uint32_t);
   }

   if (final_res == 0)
   {
      if (persistData == TRUE)
      {
         persistResult = AcdbCmdIsPersistenceSupported(&persistanceSupported);
         if (persistResult == ACDB_SUCCESS)
         {
            if (persistanceSupported == TRUE)
            {
               final_res = AcdbCmdSaveDeltaFileData();
               if (final_res != ACDB_SUCCESS)
               {
                  ACDB_DEBUG_LOG("[ACPH Online]->[batch_set_acdb_data]->Unable to save delta file data\n");
               }
            }
         }
      }
   }
   return final_res;
}

/**
 * FUNCTION : query_online_version
 *
 * DESCRIPTION : retrieve the version of ACPH Online
 *
 * DEPENDENCIES : ACPH needs to be initialized
 *
 * PARAMS:
 *   req_buf_ptr - pointer to request buffer
 *   resp_buf_ptr - pointer to response buffer
 *   resp_buf_length - length of the response buffer
 *
 * INPUT: None
 *
 * RETURN VALUE : None
 *
 * SIDE EFFECTS : None
 */
static int32_t query_online_version(uint8_t *req_buf_ptr,
   uint32_t req_buf_len,
   uint8_t *resp_buf_ptr,
   uint32_t resp_buf_length,
   uint32_t *resp_buf_bytes_filled
   )
{
   UNUSED(req_buf_len);
   UNUSED(*req_buf_ptr);

   if ((NULL == resp_buf_ptr) || (resp_buf_length < 2 * sizeof(uint32_t)))
   {
      return ACPH_ERR_OUT_OF_BUFFER_SIZE;
   }
   else
   {
      ACPH_CMD_QUERY_ONLINE_VERSION_rsp rsp;
      rsp.online_major_version = ACPH_SERVICE_MAJOR_VERSION_1;
      rsp.online_minor_version = ACPH_SERVICE_MINOR_VERSION_1;
      ACDB_MEM_CPY((void*)(resp_buf_ptr), sizeof(ACPH_CMD_QUERY_ONLINE_VERSION_rsp), (void*)&rsp, sizeof(ACPH_CMD_QUERY_ONLINE_VERSION_rsp));
      *resp_buf_bytes_filled = sizeof(ACPH_CMD_QUERY_ONLINE_VERSION_rsp);
      return ACPH_SUCCESS;
   }
}

/**
 * FUNCTION : support_acdb_persistence
 *
 * DESCRIPTION : Get information of acdb persistence support.
 *
 * DEPENDENCIES : ACPH needs to be initialized
 *
 * PARAMS:
 *   req_buf_ptr - pointer to request buffer
 *   resp_buf_ptr - pointer to response buffer
 *   resp_buf_length - length of the response buffer
 *
 * INPUT: None
 *
 * RETURN VALUE :
 *      32-bit ACPH BUFFER SIZE
 *
 * SIDE EFFECTS : None
 */
static int32_t support_acdb_persistence(uint8_t *req_buf_ptr,
   uint32_t req_buf_len,
   uint8_t *resp_buf_ptr,
   uint32_t resp_buf_length,
   uint32_t *resp_buf_bytes_filled
   )
{
   int32_t result = ACPH_SUCCESS;
   uint32_t response = FALSE;

   UNUSED(req_buf_len);
   UNUSED(*req_buf_ptr);
   UNUSED(resp_buf_length);

   result = AcdbCmdIsPersistenceSupported(&response);

   if (result == ACPH_SUCCESS)
   {
      ACDB_MEM_CPY((void*)(resp_buf_ptr), sizeof(response), (void*)&response, sizeof(response));
      *resp_buf_bytes_filled = sizeof(response);
   }

   return result;
}

/**
 * FUNCTION : delete_delta_acdb_files
 *
 * DESCRIPTION : Deletes all the delta acdb files present.
 *
 * DEPENDENCIES : ACPH needs to be initialized
 *
 * PARAMS:
 *   req_buf_ptr - pointer to request buffer
 *   resp_buf_ptr - pointer to response buffer
 *   resp_buf_length - length of the response buffer
 *
 * INPUT: None
 *
 * RETURN VALUE :
 *      32-bit ACPH BUFFER SIZE
 *
 * SIDE EFFECTS : None
 */
static int32_t delete_delta_acdb_files(uint8_t *req_buf_ptr,
   uint32_t req_buf_len,
   uint8_t *resp_buf_ptr,
   uint32_t resp_buf_length,
   uint32_t *resp_buf_bytes_filled
   )
{
   int32_t result = ACPH_SUCCESS;
   int32_t response = ACDB_ERROR;

   UNUSED(req_buf_len);
   UNUSED(*req_buf_ptr);
   UNUSED(resp_buf_length);

   result = AcdbCmdDeleteAllDeltaFiles(&response);

   if (result == ACPH_SUCCESS)
   {
      ACDB_MEM_CPY((void*)(resp_buf_ptr), sizeof(response), (void*)&response, sizeof(response));
      *resp_buf_bytes_filled = sizeof(response);
   }

   return result;
}

/*
   ----------------------------------
   | Externalized Function Definitions    |
   ----------------------------------
   */
/**
 * FUNCTION : acph_online_ioctl
 *
 * DESCRIPTION : acph online function call
 *
 * DEPENDENCIES : NONE
 *
 * PARAMS:
 *   nCommandId - command Id;
 *   req_buf_ptr - pointer to request buffer
 *   resp_buf_ptr - pointer to response buffer
 *   resp_buf_length - length of the response buffer
 *
 * RETURN VALUE : ACPH_SUCCESS/ACPH_FAILURE/ACPH_ERR_INVALID_COMMAND
 *
 * SIDE EFFECTS : None
 */
int32_t acph_online_ioctl(uint16_t nCommandId,
   uint8_t *req_buf_ptr,
   uint32_t req_buf_len,
   uint8_t *resp_buf_ptr,
   uint32_t resp_buf_len,
   uint32_t *resp_buf_bytes_filled
   )
{
   int32_t result = ACPH_SUCCESS;

   int32_t(*fcnPtr)(uint8_t *req_buf_ptr,
      uint32_t req_buf_len,
      uint8_t *resp_buf_ptr,
      uint32_t resp_buf_length,
      uint32_t *resp_buf_bytes_filled
      ) = NULL;

   switch (nCommandId)
   {
   case ACPH_CMD_GET_TARGET_VERSION:
      fcnPtr = get_target_version;
      break;
   case ACPH_CMD_CHECK_CONNECTION:
      fcnPtr = check_connection;
      break;
   case ACPH_CMD_QUERY_MAX_BUFFER_LENGTH:
      fcnPtr = query_max_buffer_length;
      break;
   case ACPH_CMD_GET_ACDB_FILES_INFO:
      fcnPtr = get_acdb_files_info;
      break;
   case ACPH_CMD_GET_ACDB_FILE:
      fcnPtr = get_acdb_file;
      break;
   case ACPH_CMD_GET_NO_OF_TBL_ENTRIES_ON_HEAP:
      fcnPtr = get_no_of_tbl_entries_on_heap;
      break;
   case ACPH_CMD_GET_TBL_ENTRIES_ON_HEAP:
      fcnPtr = get_tbl_entries_on_heap;
      break;
   case ACPH_CMD_GET_ACDB_DATA:
      fcnPtr = get_acdb_data;
      break;
   case ACPH_CMD_SET_ACDB_DATA:
      fcnPtr = set_acdb_data;
      break;
   case ACPH_CMD_QUERY_ONLINE_VERSION:
      fcnPtr = query_online_version;
      break;
   case ACPH_CMD_IS_PERSISTENCE_SUPPORTED:
      fcnPtr = support_acdb_persistence;
      break;
   case ACPH_CMD_DELETE_DELTA_ACDB_FILES:
      fcnPtr = delete_delta_acdb_files;
      break;
   case ACPH_CMD_BATCH_SET_ACDB_DATA:
      fcnPtr = batch_set_acdb_data;
      break;
   default:
      result = ACPH_ERR_INVALID_COMMAND;
   }
   //ACDB_DEBUG_LOG("ACPH Online:: CommandID: %x \n", nCommandId);
   if (result == ACPH_SUCCESS)
   {
      result = fcnPtr(req_buf_ptr,
         req_buf_len,
         resp_buf_ptr,
         resp_buf_len,
         resp_buf_bytes_filled);
   }

   return result;
}

/**
 * FUNCTION : acph_online_init
 *
 * DESCRIPTION : Initialize online calibration
 *
 * DEPENDENCIES : NONE
 *
 * PARAMS:
 *   req_buf_ptr - pointer to request buffer
 *   resp_buf_ptr - pointer to response buffer
 *   resp_buf_length - length of the response buffer
 *
 * RETURN VALUE : ACPH_SUCCESS or ACPH_FAILURE
 *
 * SIDE EFFECTS : None
 */
int32_t acph_online_init(void)
{
   int32_t result = ACPH_SUCCESS;

   result = acph_register_command(ACPH_ONLINE_REG_SERVICEID, acph_online_ioctl);
   if (result != ACPH_SUCCESS)
   {
      ACDB_DEBUG_LOG("[acph_online_intf]->acph_online_init->register command[acph_online_ioctl] failed\n");
      goto end;
   }

end:
   return result;
}

/**
 * FUNCTION : acph_online_deinit
 *
 * DESCRIPTION : De-initialize online calibration
 *
 * DEPENDENCIES : NONE
 *
 * PARAMS:
 *   req_buf_ptr - pointer to request buffer
 *   resp_buf_ptr - pointer to response buffer
 *   resp_buf_length - length of the response buffer
 *
 * RETURN VALUE : ACPH_SUCCESS or ACPH_FAILURE
 *
 * SIDE EFFECTS : None
 */
int32_t acph_online_deinit(void)
{
   int32_t result = ACPH_SUCCESS;

   result = acph_deregister_command(ACPH_ONLINE_REG_SERVICEID);
   if (result != ACPH_SUCCESS)
   {
      ACDB_DEBUG_LOG("[acph_online_intf]->acph_online_deinit->unregister command[acph_online_ioctl] failed\n");
      goto end;
   }

end:
   return result;
}