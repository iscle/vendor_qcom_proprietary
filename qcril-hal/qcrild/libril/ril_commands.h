/*
* Copyright (c) 2018 Qualcomm Technologies, Inc.
* All Rights Reserved.
* Confidential and Proprietary - Qualcomm Technologies, Inc.
*
* Not a Contribution.
* Apache license notifications and license are retained
* for attribution purposes only.
*/
/* //device/libs/telephony/ril_commands.h
**
** Copyright 2006, The Android Open Source Project
**
** Licensed under the Apache License, Version 2.0 (the "License");
** you may not use this file except in compliance with the License.
** You may obtain a copy of the License at
**
**     http://www.apache.org/licenses/LICENSE-2.0
**
** Unless required by applicable law or agreed to in writing, software
** distributed under the License is distributed on an "AS IS" BASIS,
** WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
** See the License for the specific language governing permissions and
** limitations under the License.
*/
#ifndef QMI_RIL_UTF
    {0, NULL},                   //none
    {RIL_REQUEST_GET_SIM_STATUS, NULL},
    {RIL_REQUEST_ENTER_SIM_PIN, NULL},
    {RIL_REQUEST_ENTER_SIM_PUK, NULL},
    {RIL_REQUEST_ENTER_SIM_PIN2, NULL},
    {RIL_REQUEST_ENTER_SIM_PUK2, NULL},
    {RIL_REQUEST_CHANGE_SIM_PIN, NULL},
    {RIL_REQUEST_CHANGE_SIM_PIN2, NULL},
    {RIL_REQUEST_ENTER_NETWORK_DEPERSONALIZATION, NULL},
    {RIL_REQUEST_GET_CURRENT_CALLS, radio::getCurrentCallsResponse},
    {RIL_REQUEST_DIAL, NULL},
    {RIL_REQUEST_GET_IMSI, NULL},
    {RIL_REQUEST_HANGUP, NULL},
    {RIL_REQUEST_HANGUP_WAITING_OR_BACKGROUND, NULL},
    {RIL_REQUEST_HANGUP_FOREGROUND_RESUME_BACKGROUND, NULL},
    {RIL_REQUEST_SWITCH_WAITING_OR_HOLDING_AND_ACTIVE, NULL},
    {RIL_REQUEST_CONFERENCE, NULL},
    {RIL_REQUEST_UDUB, NULL},
    {RIL_REQUEST_LAST_CALL_FAIL_CAUSE, NULL},
    {RIL_REQUEST_SIGNAL_STRENGTH, radio::getSignalStrengthResponse},
    {RIL_REQUEST_VOICE_REGISTRATION_STATE, radio::getVoiceRegistrationStateResponse},
    {RIL_REQUEST_DATA_REGISTRATION_STATE, radio::getDataRegistrationStateResponse},
    {RIL_REQUEST_OPERATOR, radio::getOperatorResponse},
    {RIL_REQUEST_RADIO_POWER, radio::setRadioPowerResponse},
    {RIL_REQUEST_DTMF, NULL},
    {RIL_REQUEST_SEND_SMS, radio::sendSmsResponse},
    {RIL_REQUEST_SEND_SMS_EXPECT_MORE, radio::sendSMSExpectMoreResponse},
    {RIL_REQUEST_SETUP_DATA_CALL, radio::setupDataCallResponse},
    {RIL_REQUEST_SIM_IO, NULL},
    {RIL_REQUEST_SEND_USSD, radio::sendUssdResponse},
    {RIL_REQUEST_CANCEL_USSD, radio::cancelPendingUssdResponse},
    {RIL_REQUEST_GET_CLIR, NULL},
    {RIL_REQUEST_SET_CLIR, NULL},
    {RIL_REQUEST_QUERY_CALL_FORWARD_STATUS, NULL},
    {RIL_REQUEST_SET_CALL_FORWARD, NULL},
    {RIL_REQUEST_QUERY_CALL_WAITING, NULL},
    {RIL_REQUEST_SET_CALL_WAITING, NULL},
    {RIL_REQUEST_SMS_ACKNOWLEDGE, NULL},
    {RIL_REQUEST_GET_IMEI, NULL},
    {RIL_REQUEST_GET_IMEISV, NULL},
    {RIL_REQUEST_ANSWER, NULL},
    {RIL_REQUEST_DEACTIVATE_DATA_CALL, radio::deactivateDataCallResponse},
    {RIL_REQUEST_QUERY_FACILITY_LOCK, NULL},
    {RIL_REQUEST_SET_FACILITY_LOCK, NULL},
    {RIL_REQUEST_CHANGE_BARRING_PASSWORD, NULL},
    {RIL_REQUEST_QUERY_NETWORK_SELECTION_MODE, radio::getNetworkSelectionModeResponse},
    {RIL_REQUEST_SET_NETWORK_SELECTION_AUTOMATIC, radio::setNetworkSelectionModeAutomaticResponse},
    {RIL_REQUEST_SET_NETWORK_SELECTION_MANUAL, radio::setNetworkSelectionModeManualResponse},
    {RIL_REQUEST_QUERY_AVAILABLE_NETWORKS , radio::getAvailableNetworksResponse},
    {RIL_REQUEST_DTMF_START, NULL},
    {RIL_REQUEST_DTMF_STOP, NULL},
    {RIL_REQUEST_BASEBAND_VERSION, radio::getBasebandVersionResponse},
    {RIL_REQUEST_SEPARATE_CONNECTION, radio::separateConnectionResponse},
    {RIL_REQUEST_SET_MUTE, radio::setMuteResponse},
    {RIL_REQUEST_GET_MUTE, radio::getMuteResponse},
    {RIL_REQUEST_QUERY_CLIP, NULL},
    {RIL_REQUEST_LAST_DATA_CALL_FAIL_CAUSE, NULL},
    {RIL_REQUEST_DATA_CALL_LIST, radio::getDataCallListResponse},
    {RIL_REQUEST_RESET_RADIO, NULL},
    {RIL_REQUEST_OEM_HOOK_RAW, radio::sendRequestRawResponse},
    {RIL_REQUEST_OEM_HOOK_STRINGS, radio::sendRequestStringsResponse},
    {RIL_REQUEST_SCREEN_STATE, radio::sendDeviceStateResponse},   // Note the response function is different.
    {RIL_REQUEST_SET_SUPP_SVC_NOTIFICATION, NULL},
    {RIL_REQUEST_WRITE_SMS_TO_SIM, NULL},
    {RIL_REQUEST_DELETE_SMS_ON_SIM, NULL},
    {RIL_REQUEST_SET_BAND_MODE, radio::setBandModeResponse},
    {RIL_REQUEST_QUERY_AVAILABLE_BAND_MODE, radio::getAvailableBandModesResponse},
    {RIL_REQUEST_STK_GET_PROFILE, NULL},
    {RIL_REQUEST_STK_SET_PROFILE, NULL},
    {RIL_REQUEST_STK_SEND_ENVELOPE_COMMAND, NULL},
    {RIL_REQUEST_STK_SEND_TERMINAL_RESPONSE, NULL},
    {RIL_REQUEST_STK_HANDLE_CALL_SETUP_REQUESTED_FROM_SIM, NULL},
    {RIL_REQUEST_EXPLICIT_CALL_TRANSFER, radio::explicitCallTransferResponse},
    {RIL_REQUEST_SET_PREFERRED_NETWORK_TYPE, radio::setPreferredNetworkTypeResponse},
    {RIL_REQUEST_GET_PREFERRED_NETWORK_TYPE, radio::getPreferredNetworkTypeResponse},
    {RIL_REQUEST_GET_NEIGHBORING_CELL_IDS, radio::getNeighboringCidsResponse},
    {RIL_REQUEST_SET_LOCATION_UPDATES, radio::setLocationUpdatesResponse},
    {RIL_REQUEST_CDMA_SET_SUBSCRIPTION_SOURCE, radio::setCdmaSubscriptionSourceResponse},
    {RIL_REQUEST_CDMA_SET_ROAMING_PREFERENCE, radio::setCdmaRoamingPreferenceResponse},
    {RIL_REQUEST_CDMA_QUERY_ROAMING_PREFERENCE, radio::getCdmaRoamingPreferenceResponse},
    {RIL_REQUEST_SET_TTY_MODE, radio::setTTYModeResponse},
    {RIL_REQUEST_QUERY_TTY_MODE, radio::getTTYModeResponse},
    {RIL_REQUEST_CDMA_SET_PREFERRED_VOICE_PRIVACY_MODE, radio::setPreferredVoicePrivacyResponse},
    {RIL_REQUEST_CDMA_QUERY_PREFERRED_VOICE_PRIVACY_MODE, radio::getPreferredVoicePrivacyResponse},
    {RIL_REQUEST_CDMA_FLASH, NULL},
    {RIL_REQUEST_CDMA_BURST_DTMF, NULL},
    {RIL_REQUEST_CDMA_VALIDATE_AND_WRITE_AKEY, NULL},
    {RIL_REQUEST_CDMA_SEND_SMS, radio::sendCdmaSmsResponse},
    {RIL_REQUEST_CDMA_SMS_ACKNOWLEDGE, NULL},
    {RIL_REQUEST_GSM_GET_BROADCAST_SMS_CONFIG, NULL},
    {RIL_REQUEST_GSM_SET_BROADCAST_SMS_CONFIG, NULL},
    {RIL_REQUEST_GSM_SMS_BROADCAST_ACTIVATION, NULL},
    {RIL_REQUEST_CDMA_GET_BROADCAST_SMS_CONFIG, NULL},
    {RIL_REQUEST_CDMA_SET_BROADCAST_SMS_CONFIG, NULL},
    {RIL_REQUEST_CDMA_SMS_BROADCAST_ACTIVATION, NULL},
    {RIL_REQUEST_CDMA_SUBSCRIPTION, radio::getCDMASubscriptionResponse},
    {RIL_REQUEST_CDMA_WRITE_SMS_TO_RUIM, NULL},
    {RIL_REQUEST_CDMA_DELETE_SMS_ON_RUIM, NULL},
    {RIL_REQUEST_DEVICE_IDENTITY, radio::getDeviceIdentityResponse},
    {RIL_REQUEST_EXIT_EMERGENCY_CALLBACK_MODE, radio::exitEmergencyCallbackModeResponse},
    {RIL_REQUEST_GET_SMSC_ADDRESS, NULL},
    {RIL_REQUEST_SET_SMSC_ADDRESS, NULL},
    {RIL_REQUEST_REPORT_SMS_MEMORY_STATUS, NULL},
    {RIL_REQUEST_REPORT_STK_SERVICE_IS_RUNNING, NULL},
    {RIL_REQUEST_CDMA_GET_SUBSCRIPTION_SOURCE, radio::getCdmaSubscriptionSourceResponse},
    {RIL_REQUEST_ISIM_AUTHENTICATION, NULL},
    {RIL_REQUEST_ACKNOWLEDGE_INCOMING_GSM_SMS_WITH_PDU, radio::acknowledgeIncomingGsmSmsWithPduResponse},
    {RIL_REQUEST_STK_SEND_ENVELOPE_WITH_STATUS, NULL},
    {RIL_REQUEST_VOICE_RADIO_TECH, radio::getVoiceRadioTechnologyResponse},
    {RIL_REQUEST_GET_CELL_INFO_LIST, radio::getCellInfoListResponse},
    {RIL_REQUEST_SET_UNSOL_CELL_INFO_LIST_RATE, radio::setCellInfoListRateResponse},
    {RIL_REQUEST_SET_INITIAL_ATTACH_APN, radio::setInitialAttachApnResponse},
    {RIL_REQUEST_IMS_REGISTRATION_STATE, NULL},
    {RIL_REQUEST_IMS_SEND_SMS, radio::sendImsSmsResponse},
    {RIL_REQUEST_SIM_TRANSMIT_APDU_BASIC, NULL},
    {RIL_REQUEST_SIM_OPEN_CHANNEL, NULL},
    {RIL_REQUEST_SIM_CLOSE_CHANNEL, NULL},
    {RIL_REQUEST_SIM_TRANSMIT_APDU_CHANNEL, NULL},
    {RIL_REQUEST_NV_READ_ITEM, radio::nvReadItemResponse},
    {RIL_REQUEST_NV_WRITE_ITEM, radio::nvWriteItemResponse},
    {RIL_REQUEST_NV_WRITE_CDMA_PRL, radio::nvWriteCdmaPrlResponse},
    {RIL_REQUEST_NV_RESET_CONFIG, radio::nvResetConfigResponse},
    {RIL_REQUEST_SET_UICC_SUBSCRIPTION, radio::setUiccSubscriptionResponse},
    {RIL_REQUEST_ALLOW_DATA, radio::setDataAllowedResponse},
    {RIL_REQUEST_GET_HARDWARE_CONFIG, radio::getHardwareConfigResponse},
    {RIL_REQUEST_SIM_AUTHENTICATION, NULL},
    {RIL_REQUEST_GET_DC_RT_INFO, NULL},
    {RIL_REQUEST_SET_DC_RT_INFO_RATE, NULL},
    {RIL_REQUEST_SET_DATA_PROFILE, radio::setDataProfileResponse},
    {RIL_REQUEST_SHUTDOWN, radio::requestShutdownResponse},
    {RIL_REQUEST_GET_RADIO_CAPABILITY, radio::getRadioCapabilityResponse},
    {RIL_REQUEST_SET_RADIO_CAPABILITY, radio::setRadioCapabilityResponse},
    {RIL_REQUEST_START_LCE, radio::startLceServiceResponse},
    {RIL_REQUEST_STOP_LCE, radio::stopLceServiceResponse},
    {RIL_REQUEST_PULL_LCEDATA, radio::pullLceDataResponse},
    {RIL_REQUEST_GET_ACTIVITY_INFO, radio::getModemActivityInfoResponse},
    {RIL_REQUEST_SET_CARRIER_RESTRICTIONS, radio::setAllowedCarriersResponse},
    {RIL_REQUEST_GET_CARRIER_RESTRICTIONS, radio::getAllowedCarriersResponse},
    {RIL_REQUEST_SEND_DEVICE_STATE, radio::sendDeviceStateResponse},
    {RIL_REQUEST_SET_UNSOLICITED_RESPONSE_FILTER, radio::setIndicationFilterResponse},
    {RIL_REQUEST_SET_SIM_CARD_POWER, NULL},
    {RIL_REQUEST_SET_CARRIER_INFO_IMSI_ENCRYPTION, radio::setCarrierInfoForImsiEncryptionResponse},
    {RIL_REQUEST_START_NETWORK_SCAN, radio::startNetworkScanResponse},
    {RIL_REQUEST_STOP_NETWORK_SCAN, radio::stopNetworkScanResponse},
    {RIL_REQUEST_START_KEEPALIVE, radio::startKeepaliveResponse},
    {RIL_REQUEST_STOP_KEEPALIVE, radio::stopKeepaliveResponse},
#endif
