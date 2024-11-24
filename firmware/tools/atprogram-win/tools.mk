
.PHONY: flash-write fuse-read fuse-write

DIR_TOOLS_ATPROGRAM = tools/atprogram-win

flash-write: $(ELF)
	powershell.exe $(DIR_TOOLS_ATPROGRAM)/flash-write.ps1

fuse-read:
	powershell.exe $(DIR_TOOLS_ATPROGRAM)/fuse-read.ps1

fuse-write:
	powershell.exe $(DIR_TOOLS_ATPROGRAM)/fuse-write.ps1
