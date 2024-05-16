# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources(tmag5273.c)
zephyr_library_sources_ifdef(CONFIG_TMAG5273_TRIGGER tmag5273_trigger.c)
