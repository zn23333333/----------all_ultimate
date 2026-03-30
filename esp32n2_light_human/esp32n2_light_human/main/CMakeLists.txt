idf_build_get_property(target IDF_TARGET)
set(priv_requires nvs_flash esp_wifi esp_event esp_netif lwip driver)

if(${target} STREQUAL "linux")
    # Keep Linux dependencies explicit; base list already includes required libs.
endif()

idf_component_register(SRCS "udp_client.c"
                    PRIV_REQUIRES ${priv_requires}
                    INCLUDE_DIRS ".")
