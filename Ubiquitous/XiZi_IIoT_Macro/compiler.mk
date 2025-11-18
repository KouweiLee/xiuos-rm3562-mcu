ifeq ($(COMPILE_TYPE), COMPILE_MUSL) 
SRC_DIR_TEMP := $(MUSL_DIR)
else ifeq ($(COMPILE_TYPE), COMPILE_LWIP) 
SRC_DIR_TEMP := $(LWIP_DIR)
else ifeq ($(COMPILE_TYPE), COMPILE_MONGOOSE)
SRC_DIR_TEMP := $(MONGOOSE_DIR)
else
SRC_DIR_TEMP := $(SRC_DIR) 
endif

SRC_DIR :=
MUSL_DIR :=
LWIP_DIR :=
MONGOOSE_DIR :=

ifeq ($(USE_APP_INCLUDEPATH), y)
	include $(KERNEL_ROOT)/path_app.mk
else 
	include $(KERNEL_ROOT)/path_kernel.mk
endif
export CPPPATHS := $(KERNELPATHS)

CUR_DIR :=$(shell pwd)

CFLAGS += $(CPPPATHS)
AFLAGS += $(CPPPATHS)
CXXFLAGS += $(CPPPATHS)

CFLAGS += $(DEFINES)
AFLAGS += $(DEFINES)
CXXFLAGS += $(DEFINES)
BUILD_DIR := $(KERNEL_ROOT)/build
APP_DIR := Ubiquitous/XiZi_IIoT

OBJS_LIST_FILE := $(BUILD_DIR)/make.obj

.PHONY:COMPILER

COMPILER:
	@if [ "${SRC_DIR_TEMP}" != "" ];	then \
		for dir in $(SRC_DIR_TEMP); do \
			$(MAKE) -C $$dir; \
		done; \
	fi
	@flock -x $(OBJS_LIST_FILE) -c "echo -n '$(OBJS) ' >> $(OBJS_LIST_FILE)"

################################################
define add_c_file
$(eval COBJ := $(1:%.c=%.o)) \
$(eval COBJ := $(subst $(subst $(APP_DIR),,$(KERNEL_ROOT)),,$(COBJ))) \
$(eval LOCALC := $(addprefix $(BUILD_DIR)/,$(COBJ))) \
$(eval OBJS += $(LOCALC)) \
$(if $(strip $(LOCALC)),$(eval $(LOCALC): $(1)
	@if [ ! -d $$(@D) ]; then mkdir -p $$(@D); fi
	@echo cc $$<
	@$(CROSS_COMPILE)gcc $$(CFLAGS) -MMD -c $$< -o $$@))
endef

define add_cpp_file
$(eval COBJ := $(1:%.cpp=%.o)) \
$(eval COBJ := $(subst $(subst $(APP_DIR),,$(KERNEL_ROOT)),,$(COBJ))) \
$(eval LOCALCPP := $(addprefix $(BUILD_DIR)/,$(COBJ))) \
$(eval OBJS += $(LOCALCPP)) \
$(if $(strip $(LOCALCPP)),$(eval $(LOCALCPP): $(1)
	@if [ ! -d $$(@D) ]; then mkdir -p $$(@D); fi
	@echo cpp $$<
	@$(CROSS_COMPILE)g++ $$(CXXFLAGS) -MMD -c $$< -o $$@))
endef

define add_cc_file
$(eval COBJ := $(1:%.cc=%.o)) \
$(eval COBJ := $(subst $(subst $(APP_DIR),,$(KERNEL_ROOT)),,$(COBJ))) \
$(eval LOCALCPP := $(addprefix $(BUILD_DIR)/,$(COBJ))) \
$(eval OBJS += $(LOCALCPP)) \
$(if $(strip $(LOCALCPP)),$(eval $(LOCALCPP): $(1)
	@if [ ! -d $$(@D) ]; then mkdir -p $$(@D); fi
	@echo cpp $$<
	@$(CROSS_COMPILE)g++ $$(CXXFLAGS) -MMD -c $$< -o $$@))
endef

define add_S_file
$(eval SOBJ := $(1:%.S=%.o)) \
$(eval SOBJ := $(subst $(subst $(APP_DIR),,$(KERNEL_ROOT)),,$(SOBJ))) \
$(eval LOCALS := $(addprefix $(BUILD_DIR)/,$(SOBJ))) \
$(eval OBJS += $(LOCALS)) \
$(if $(strip $(LOCALS)),$(eval $(LOCALS): $(1)
	@if [ ! -d $$(@D) ]; then mkdir -p $$(@D); fi
	@echo asm $$<
	@$(CROSS_COMPILE)gcc $$(AFLAGS) -MMD -c $$< -o $$@))
endef

define add_a_file
$(eval SOBJ := $(1:%.a=%.a)) \
$(eval SOBJ := $(subst $(subst $(APP_DIR),,$(KERNEL_ROOT)),,$(SOBJ))) \
$(eval LOCALA := $(addprefix $(BUILD_DIR)/,$(SOBJ))) \
$(eval OBJS += $(LOCALA)) \
$(if $(strip $(LOCALA)),$(eval $(LOCALA): $(1)
	@if [ ! -d $$(@D) ]; then mkdir -p $$(@D); fi
	@echo cp $$<
	@cp $$< $$@))
endef



SRCS := $(strip $(filter %.c,$(SRC_FILES)))
$(if $(SRCS),$(foreach f,$(SRCS),$(call add_c_file,$(addprefix $(CUR_DIR)/,$(f)))))

SRCS := $(strip $(filter %.cpp,$(SRC_FILES)))
$(if $(SRCS),$(foreach f,$(SRCS),$(call add_cpp_file,$(addprefix $(CUR_DIR)/,$(f)))))

SRCS := $(strip $(filter %.cc,$(SRC_FILES)))
$(if $(SRCS),$(foreach f,$(SRCS),$(call add_cc_file,$(addprefix $(CUR_DIR)/,$(f)))))

SRCS := $(strip $(filter %.S,$(SRC_FILES)))
$(if $(SRCS),$(foreach f,$(SRCS),$(call add_S_file,$(addprefix $(CUR_DIR)/,$(f)))))

SRCS := $(strip $(filter %.a,$(SRC_FILES)))
$(if $(SRCS),$(foreach f,$(SRCS),$(call add_a_file,$(addprefix $(CUR_DIR)/,$(f)))))

COMPILER: $(OBJS)

-include $(OBJS:%.o=%.d)
