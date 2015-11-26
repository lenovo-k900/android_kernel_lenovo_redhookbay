/*************************************************************************/ /*!
@File
@Title          Server bridge for cmm
@Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
@Description    Implements the server side of the bridge for cmm
@License        Dual MIT/GPLv2

The contents of this file are subject to the MIT license as set out below.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

Alternatively, the contents of this file may be used under the terms of
the GNU General Public License Version 2 ("GPL") in which case the provisions
of GPL are applicable instead of those above.

If you wish to allow use of your version of this file only under the terms of
GPL, and not to allow others to use your version of this file under the terms
of the MIT license, indicate your decision by deleting the provisions above
and replace them with the notice and other provisions required by GPL as set
out in the file called "GPL-COPYING" included in this distribution. If you do
not delete the provisions above, a recipient may use your version of this file
under the terms of either the MIT license or GPL.

This License is also included in this distribution in the file called
"MIT-COPYING".

EXCEPT AS OTHERWISE STATED IN A NEGOTIATED AGREEMENT: (A) THE SOFTWARE IS
PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
PURPOSE AND NONINFRINGEMENT; AND (B) IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/ /**************************************************************************/

#include <stddef.h>
#include <asm/uaccess.h>

#include "img_defs.h"

#include "pmr.h"


#include "common_cmm_bridge.h"

#include "allocmem.h"
#include "pvr_debug.h"
#include "connection_server.h"
#include "pvr_bridge.h"
#include "rgx_bridge.h"
#include "srvcore.h"
#include "handle.h"

#if defined (SUPPORT_AUTH)
#include "osauth.h"
#endif

#include <linux/slab.h>

/* ***************************************************************************
 * Bridge proxy functions
 */

static PVRSRV_ERROR
PMRUnwritePMPageListResManProxy(IMG_HANDLE hResmanItem)
{
	PVRSRV_ERROR eError;

	eError = ResManFreeResByPtr(hResmanItem);

	/* Freeing a resource should never fail... */
	PVR_ASSERT((eError == PVRSRV_OK) || (eError == PVRSRV_ERROR_RETRY));

	return eError;
}



/* ***************************************************************************
 * Server-side bridge entry points
 */
 
static IMG_INT
PVRSRVBridgePMRWritePMPageList(IMG_UINT32 ui32BridgeID,
					 PVRSRV_BRIDGE_IN_PMRWRITEPMPAGELIST *psPMRWritePMPageListIN,
					 PVRSRV_BRIDGE_OUT_PMRWRITEPMPAGELIST *psPMRWritePMPageListOUT,
					 CONNECTION_DATA *psConnection)
{
	PMR * psPageListPMRInt = IMG_NULL;
	IMG_HANDLE hPageListPMRInt2 = IMG_NULL;
	PMR * psReferencePMRInt = IMG_NULL;
	IMG_HANDLE hReferencePMRInt2 = IMG_NULL;
	PMR_PAGELIST * psPageListInt = IMG_NULL;
	IMG_HANDLE hPageListInt2 = IMG_NULL;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID, PVRSRV_BRIDGE_CMM_PMRWRITEPMPAGELIST);





				{
					/* Look up the address from the handle */
					psPMRWritePMPageListOUT->eError =
						PVRSRVLookupHandle(psConnection->psHandleBase,
											(IMG_HANDLE *) &hPageListPMRInt2,
											psPMRWritePMPageListIN->hPageListPMR,
											PVRSRV_HANDLE_TYPE_PHYSMEM_PMR);
					if(psPMRWritePMPageListOUT->eError != PVRSRV_OK)
					{
						goto PMRWritePMPageList_exit;
					}

					/* Look up the data from the resman address */
					psPMRWritePMPageListOUT->eError = ResManFindPrivateDataByPtr(hPageListPMRInt2, (IMG_VOID **) &psPageListPMRInt);

					if(psPMRWritePMPageListOUT->eError != PVRSRV_OK)
					{
						goto PMRWritePMPageList_exit;
					}
				}

				{
					/* Look up the address from the handle */
					psPMRWritePMPageListOUT->eError =
						PVRSRVLookupHandle(psConnection->psHandleBase,
											(IMG_HANDLE *) &hReferencePMRInt2,
											psPMRWritePMPageListIN->hReferencePMR,
											PVRSRV_HANDLE_TYPE_PHYSMEM_PMR);
					if(psPMRWritePMPageListOUT->eError != PVRSRV_OK)
					{
						goto PMRWritePMPageList_exit;
					}

					/* Look up the data from the resman address */
					psPMRWritePMPageListOUT->eError = ResManFindPrivateDataByPtr(hReferencePMRInt2, (IMG_VOID **) &psReferencePMRInt);

					if(psPMRWritePMPageListOUT->eError != PVRSRV_OK)
					{
						goto PMRWritePMPageList_exit;
					}
				}

	psPMRWritePMPageListOUT->eError =
		PMRWritePMPageList(
					psPageListPMRInt,
					psPMRWritePMPageListIN->uiTableOffset,
					psPMRWritePMPageListIN->uiTableLength,
					psReferencePMRInt,
					psPMRWritePMPageListIN->ui32Log2PageSize,
					&psPageListInt,
					&psPMRWritePMPageListOUT->ui64CheckSum);
	/* Exit early if bridged call fails */
	if(psPMRWritePMPageListOUT->eError != PVRSRV_OK)
	{
		goto PMRWritePMPageList_exit;
	}

	/* Create a resman item and overwrite the handle with it */
	hPageListInt2 = ResManRegisterRes(psConnection->hResManContext,
												RESMAN_TYPE_PMR_PAGELIST,
												psPageListInt,
												/* FIXME: how can we avoid this cast? */
												(RESMAN_FREE_FN)&PMRUnwritePMPageList);
	if (hPageListInt2 == IMG_NULL)
	{
		psPMRWritePMPageListOUT->eError = PVRSRV_ERROR_UNABLE_TO_REGISTER_RESOURCE;
		goto PMRWritePMPageList_exit;
	}
	psPMRWritePMPageListOUT->eError = PVRSRVAllocHandle(psConnection->psHandleBase,
							&psPMRWritePMPageListOUT->hPageList,
							(IMG_HANDLE) hPageListInt2,
							PVRSRV_HANDLE_TYPE_PHYSMEM_PMR_PAGELIST,
							PVRSRV_HANDLE_ALLOC_FLAG_NONE
							);
	if (psPMRWritePMPageListOUT->eError != PVRSRV_OK)
	{
		goto PMRWritePMPageList_exit;
	}


PMRWritePMPageList_exit:
	if (psPMRWritePMPageListOUT->eError != PVRSRV_OK)
	{
		/* If we have a valid resman item we should undo the bridge function by freeing the resman item */
		if (hPageListInt2)
		{
			PVRSRV_ERROR eError = ResManFreeResByPtr(hPageListInt2);

			/* Freeing a resource should never fail... */
			PVR_ASSERT((eError == PVRSRV_OK) || (eError == PVRSRV_ERROR_RETRY));
		}
		else if (psPageListInt)
		{
			PMRUnwritePMPageList(psPageListInt);
		}
	}


	return 0;
}

static IMG_INT
PVRSRVBridgePMRWriteVFPPageList(IMG_UINT32 ui32BridgeID,
					 PVRSRV_BRIDGE_IN_PMRWRITEVFPPAGELIST *psPMRWriteVFPPageListIN,
					 PVRSRV_BRIDGE_OUT_PMRWRITEVFPPAGELIST *psPMRWriteVFPPageListOUT,
					 CONNECTION_DATA *psConnection)
{
	PMR * psFreeListPMRInt = IMG_NULL;
	IMG_HANDLE hFreeListPMRInt2 = IMG_NULL;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID, PVRSRV_BRIDGE_CMM_PMRWRITEVFPPAGELIST);





				{
					/* Look up the address from the handle */
					psPMRWriteVFPPageListOUT->eError =
						PVRSRVLookupHandle(psConnection->psHandleBase,
											(IMG_HANDLE *) &hFreeListPMRInt2,
											psPMRWriteVFPPageListIN->hFreeListPMR,
											PVRSRV_HANDLE_TYPE_PHYSMEM_PMR);
					if(psPMRWriteVFPPageListOUT->eError != PVRSRV_OK)
					{
						goto PMRWriteVFPPageList_exit;
					}

					/* Look up the data from the resman address */
					psPMRWriteVFPPageListOUT->eError = ResManFindPrivateDataByPtr(hFreeListPMRInt2, (IMG_VOID **) &psFreeListPMRInt);

					if(psPMRWriteVFPPageListOUT->eError != PVRSRV_OK)
					{
						goto PMRWriteVFPPageList_exit;
					}
				}

	psPMRWriteVFPPageListOUT->eError =
		PMRWriteVFPPageList(
					psFreeListPMRInt,
					psPMRWriteVFPPageListIN->uiTableOffset,
					psPMRWriteVFPPageListIN->uiTableLength,
					psPMRWriteVFPPageListIN->ui32TableBase,
					psPMRWriteVFPPageListIN->ui32Log2PageSize);



PMRWriteVFPPageList_exit:

	return 0;
}

static IMG_INT
PVRSRVBridgePMRUnwritePMPageList(IMG_UINT32 ui32BridgeID,
					 PVRSRV_BRIDGE_IN_PMRUNWRITEPMPAGELIST *psPMRUnwritePMPageListIN,
					 PVRSRV_BRIDGE_OUT_PMRUNWRITEPMPAGELIST *psPMRUnwritePMPageListOUT,
					 CONNECTION_DATA *psConnection)
{
	IMG_HANDLE hPageListInt2 = IMG_NULL;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID, PVRSRV_BRIDGE_CMM_PMRUNWRITEPMPAGELIST);





				{
					/* Look up the address from the handle */
					psPMRUnwritePMPageListOUT->eError =
						PVRSRVLookupHandle(psConnection->psHandleBase,
											(IMG_HANDLE *) &hPageListInt2,
											psPMRUnwritePMPageListIN->hPageList,
											PVRSRV_HANDLE_TYPE_PHYSMEM_PMR_PAGELIST);
					if(psPMRUnwritePMPageListOUT->eError != PVRSRV_OK)
					{
						goto PMRUnwritePMPageList_exit;
					}

				}

	psPMRUnwritePMPageListOUT->eError = PMRUnwritePMPageListResManProxy(hPageListInt2);
	/* Exit early if bridged call fails */
	if(psPMRUnwritePMPageListOUT->eError != PVRSRV_OK)
	{
		goto PMRUnwritePMPageList_exit;
	}

	psPMRUnwritePMPageListOUT->eError =
		PVRSRVReleaseHandle(psConnection->psHandleBase,
					(IMG_HANDLE) psPMRUnwritePMPageListIN->hPageList,
					PVRSRV_HANDLE_TYPE_PHYSMEM_PMR_PAGELIST);


PMRUnwritePMPageList_exit:

	return 0;
}



/* *************************************************************************** 
 * Server bridge dispatch related glue 
 */
 
PVRSRV_ERROR RegisterCMMFunctions(IMG_VOID);
IMG_VOID UnregisterCMMFunctions(IMG_VOID);

/*
 * Register all CMM functions with services
 */
PVRSRV_ERROR RegisterCMMFunctions(IMG_VOID)
{
	SetDispatchTableEntry(PVRSRV_BRIDGE_CMM_PMRWRITEPMPAGELIST, PVRSRVBridgePMRWritePMPageList);
	SetDispatchTableEntry(PVRSRV_BRIDGE_CMM_PMRWRITEVFPPAGELIST, PVRSRVBridgePMRWriteVFPPageList);
	SetDispatchTableEntry(PVRSRV_BRIDGE_CMM_PMRUNWRITEPMPAGELIST, PVRSRVBridgePMRUnwritePMPageList);

	return PVRSRV_OK;
}

/*
 * Unregister all cmm functions with services
 */
IMG_VOID UnregisterCMMFunctions(IMG_VOID)
{
}
