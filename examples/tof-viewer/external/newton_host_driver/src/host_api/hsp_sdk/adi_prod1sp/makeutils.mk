# Recurse a directory building a list of matching files in all subdirectories
rwildcard = $(wildcard $1$2) $(foreach d,$(wildcard $1*),$(call rwildcard,$d/,$2))

# Make a string all uppercase
makeUpper = $(subst a,A,$(subst b,B,$(subst c,C,$(subst d,D,$(subst e,E,$(subst f,F,$(subst g,G,$(subst h,H,$(subst i,I,$(subst j,J,$\
            $(subst k,K,$(subst l,L,$(subst m,M,$(subst n,N,$(subst o,O,$(subst p,P,$(subst q,Q,$(subst r,R,$(subst s,S,$(subst t,T,$\
		    $(subst u,U,$(subst v,V,$(subst w,W,$(subst x,X,$(subst y,Y,$(subst z,Z,$1))))))))))))))))))))))))))

ifeq ($(OS), Windows_NT)
CREATE_FILE = type nul
REMOVE_FOLDER = rmdir /s /q
else
CREATE_FILE = touch
REMOVE_FOLDER = rm -rf
endif

ifeq ($(OS), Windows_NT)
define makedir
    @if not exist "$(subst /,\,$(1))" mkdir "$(subst /,\,$(1))"
endef
else
define makedir
    @-mkdir -p "$(1)"
endef
endif
