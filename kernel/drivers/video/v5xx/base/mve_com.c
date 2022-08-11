/*
 * (C) COPYRIGHT ARM Limited. All rights reserved.
 *
 * This program is free software and is provided to you under the terms of the
 * GNU General Public License version 2 as published by the Free Software
 * Foundation, and any use by you of this program is subject to the terms
 * of such GNU licence.
 *
 * A copy of the licence is included with the program, and can also be obtained
 * from Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
 * Boston, MA  02110-1301, USA.
 *
 */

#include "mve_com.h"
#include "mve_session.h"
#include "mve_rsrc_log.h"

#include <host_interface_v1/mve_protocol_kernel.h>

#include "mve_com_host_interface_v1.h"
#include "mve_com_host_interface_v2.h"

mve_error mve_com_add_message(struct mve_session *session,
                              uint16_t code,
                              uint16_t size,
                              uint32_t *data)
{
    mve_error ret = MVE_ERROR_NONE;

    if (session->com == NULL || session->com->host_interface.add_message == NULL)
    {
        return MVE_ERROR_VERSION_MISMATCH;
    }

    ret = session->com->host_interface.add_message(session, code, size, data);

    if (MVE_ERROR_NONE != ret)
    {
        MVE_LOG_PRINT(&mve_rsrc_log_fwif, MVE_LOG_ERROR, "%p mve_com_add_message() returned error status. ret=%u, code=%u, size=%u.", session, ret, code, size);
    }

    if (MVE_MESSAGE_CODE_SWITCH != code)
    {
        session->state.idle_state = IDLE_STATE_ACTIVE;
    }

    return ret;
}

uint32_t *mve_com_get_message(struct mve_session *session,
                              struct mve_msg_header *header)
{
    uint32_t *ret = NULL;

    if (session->com == NULL || session->com->host_interface.get_message == NULL)
    {
        return NULL;
    }

    ret = session->com->host_interface.get_message(session, header);

    return ret;
}

mve_error mve_com_add_input_buffer(struct mve_session *session,
                                   mve_com_buffer *buffer,
                                   enum mve_com_buffer_type type)
{
    mve_error res;

    if (session->com == NULL || session->com->host_interface.add_input_buffer == NULL)
    {
        return MVE_ERROR_VERSION_MISMATCH;
    }

    res = session->com->host_interface.add_input_buffer(session, buffer, type);

    if (MVE_ERROR_NONE == res)
    {
        if (type != MVE_COM_BUFFER_TYPE_ROI)
        {
            session->input_buffer_count++;
        }
    }
    else
    {
        MVE_LOG_PRINT(&mve_rsrc_log_fwif, MVE_LOG_ERROR, "%p mve_com_add_input_buffer() returned error status. ret=%u.", session, res);
    }

    return res;
}

mve_error mve_com_add_output_buffer(struct mve_session *session,
                                    mve_com_buffer *buffer,
                                    enum mve_com_buffer_type type)
{
    mve_error res;

    if (session->com == NULL || session->com->host_interface.add_output_buffer == NULL)
    {
        return MVE_ERROR_VERSION_MISMATCH;
    }

    res = session->com->host_interface.add_output_buffer(session, buffer, type);

    if (MVE_ERROR_NONE == res)
    {
        if (type != MVE_COM_BUFFER_TYPE_ROI)
        {
            session->output_buffer_count++;
        }
    }
    else
    {
        MVE_LOG_PRINT(&mve_rsrc_log_fwif, MVE_LOG_ERROR, "%p mve_com_add_output_buffer() returned error status. ret=%u.", session, res);
    }

    return res;
}

mve_error mve_com_get_input_buffer(struct mve_session *session,
                                   mve_com_buffer *buffer)
{
    mve_error ret;

    if (session->com == NULL || session->com->host_interface.get_input_buffer == NULL)
    {
        return MVE_ERROR_VERSION_MISMATCH;
    }

    ret = session->com->host_interface.get_input_buffer(session, buffer);

    if (MVE_ERROR_NONE == ret)
    {
        session->input_buffer_count--;
    }
    else
    {
        MVE_LOG_PRINT(&mve_rsrc_log_fwif, MVE_LOG_ERROR, "%p mve_com_get_input_buffer() returned error status. ret=%u.", session, ret);
    }

    return ret;
}

mve_error mve_com_get_output_buffer(struct mve_session *session,
                                    mve_com_buffer *buffer)
{
    mve_error ret;

    if (session->com == NULL || session->com->host_interface.get_output_buffer == NULL)
    {
        return MVE_ERROR_VERSION_MISMATCH;
    }

    ret = session->com->host_interface.get_output_buffer(session, buffer);

    if (MVE_ERROR_NONE == ret)
    {
        session->output_buffer_count--;
    }
    else
    {
        MVE_LOG_PRINT(&mve_rsrc_log_fwif, MVE_LOG_ERROR, "%p mve_com_get_output_buffer() returned error status. ret=%u.", session, ret);
    }

    return ret;
}

mve_error mve_com_get_rpc_message(struct mve_session *session,
                                  mve_com_rpc *rpc)
{
    if (NULL == session->com || NULL == session->com->host_interface.get_rpc_message)
    {
        return MVE_ERROR_VERSION_MISMATCH;
    }

    return session->com->host_interface.get_rpc_message(session, rpc);
}

mve_error mve_com_put_rpc_message(struct mve_session *session,
                                  mve_com_rpc *rpc)
{
    if (NULL == session->com || NULL == session->com->host_interface.put_rpc_message)
    {
        return MVE_ERROR_VERSION_MISMATCH;
    }

    return session->com->host_interface.put_rpc_message(session, rpc);
}

void mve_com_delete(struct mve_session *session)
{
    MVE_RSRC_MEM_FREE(session->com);
    session->com = NULL;
}

mve_error mve_com_set_interface_version(struct mve_session *session,
                                        enum mve_fw_protocol_version version)
{
    mve_com_delete(session);

    switch (version)
    {
        case MVE_FW_PROTOCOL_VERSION_1_0:
        {
            session->com = mve_com_host_interface_v1_new();
            break;
        }

        case MVE_FW_PROTOCOL_VERSION_2_0:
        {
            session->com = mve_com_host_interface_v2_new();
            break;
        }

        default:
        {
            MVE_LOG_PRINT(&mve_rsrc_log_fwif, MVE_LOG_ERROR, "%p unsupported interface version configured. version=%u.", session, version);
            return MVE_ERROR_BAD_PARAMETER;
        }
    }

    if (session->com == NULL)
    {
        return MVE_ERROR_INSUFFICIENT_RESOURCES;
    }

    return MVE_ERROR_NONE;
}
