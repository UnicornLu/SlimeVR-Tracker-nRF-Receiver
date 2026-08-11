set(partition_overlay_dir ${CMAKE_CURRENT_LIST_DIR}/dts/partitions)
set(partition_overlay)

if(DEFINED EXTRA_DTC_OVERLAY_FILE)
  list(APPEND ${DEFAULT_IMAGE}_EXTRA_DTC_OVERLAY_FILE ${EXTRA_DTC_OVERLAY_FILE})
endif()

if(SB_CONFIG_BOARD STREQUAL "xiao_ble")
  set(partition_overlay ${partition_overlay_dir}/nrf52840_xiao.overlay)
elseif(SB_CONFIG_BOARD MATCHES "^nrf52840dongle$|^holyiot_21017$")
  set(partition_overlay ${partition_overlay_dir}/nrf52840_dongle.overlay)
elseif(SB_CONFIG_SOC_NRF52833)
  set(partition_overlay ${partition_overlay_dir}/nrf52833_uf2.overlay)
elseif(SB_CONFIG_SOC_NRF52840)
  set(partition_overlay ${partition_overlay_dir}/nrf52840_uf2.overlay)
endif()

if(partition_overlay)
  list(APPEND ${DEFAULT_IMAGE}_EXTRA_DTC_OVERLAY_FILE ${partition_overlay})
  set(${DEFAULT_IMAGE}_EXTRA_DTC_OVERLAY_FILE
      ${${DEFAULT_IMAGE}_EXTRA_DTC_OVERLAY_FILE}
      CACHE INTERNAL "")
endif()
