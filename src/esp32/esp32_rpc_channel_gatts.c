/*
 * Copyright (c) 2014-2017 Cesanta Software Limited
 * All rights reserved
 */

/*
 * RPC over GATT, (S)erver only for now.
 * See README.md for description.
 */

#include <stdlib.h>

#include "mg_rpc.h"
#include "mgos_hal.h"
#include "mgos_rpc.h"

#include "common/cs_dbg.h"

#include "esp32_bt.h"

#ifndef MGOS_RPC_CHANNEL_GATTS_MAX_FRAME_LEN
#define MGOS_RPC_CHANNEL_GATTS_MAX_FRAME_LEN 8192
#endif

/* Note: UUIDs below are in reverse, because that's how ESP wants them. */
static const esp_bt_uuid_t mos_rpc_svc_uuid = {
    .len = ESP_UUID_LEN_128,
    .uuid.uuid128 =
        {
         /* _mOS_RPC_SVC_ID_, 5f6d4f53-5f52-5043-5f53-56435f49445f */
         0x5f, 0x44, 0x49, 0x5f, 0x43, 0x56, 0x53, 0x5f, 0x43, 0x50, 0x52, 0x5f,
         0x53, 0x4f, 0x6d, 0x5f,
        },
};

static const esp_bt_uuid_t mos_rpc_data_uuid = {
    .len = ESP_UUID_LEN_128,
    .uuid.uuid128 =
        {
         /* _mOS_RPC_data___, 5f6d4f53-5f52-5043-5f64-6174615f5f5f */
         0x5f, 0x5f, 0x5f, 0x61, 0x74, 0x61, 0x64, 0x5f, 0x43, 0x50, 0x52, 0x5f,
         0x53, 0x4f, 0x6d, 0x5f,
        },
};
static uint16_t mos_rpc_data_ah;

static const esp_bt_uuid_t mos_rpc_rx_ctl_uuid = {
    .len = ESP_UUID_LEN_128,
    .uuid.uuid128 =
        {
         /* _mOS_RPC_rx_ctl_, 5f6d4f53-5f52-5043-5f72-785f63746c5f */
         0x5f, 0x6c, 0x74, 0x63, 0x5f, 0x78, 0x72, 0x5f, 0x43, 0x50, 0x52, 0x5f,
         0x53, 0x4f, 0x6d, 0x5f,
        },
};
static uint16_t mos_rpc_rx_ctl_ah;

static const esp_bt_uuid_t mos_rpc_tx_ctl_uuid = {
    .len = ESP_UUID_LEN_128,
    .uuid.uuid128 =
        {
         /* _mOS_RPC_tx_ctl_, 5f6d4f53-5f52-5043-5f74-785f63746c5f */
         0x5f, 0x6c, 0x74, 0x63, 0x5f, 0x78, 0x74, 0x5f, 0x43, 0x50, 0x52, 0x5f,
         0x53, 0x4f, 0x6d, 0x5f,
        },
};
static uint16_t mos_rpc_tx_ctl_ah;

const esp_gatts_attr_db_t mos_rpc_gatt_db[7] = {
    {
     .attr_control = {.auto_rsp = ESP_GATT_AUTO_RSP},
     .att_desc =
         {
          .uuid_length = ESP_UUID_LEN_16,
          .uuid_p = (uint8_t *) &primary_service_uuid,
          .perm = ESP_GATT_PERM_READ,
          .max_length = ESP_UUID_LEN_128,
          .length = ESP_UUID_LEN_128,
          .value = (uint8_t *) mos_rpc_svc_uuid.uuid.uuid128,
         },
    },

    /* data */
    {{ESP_GATT_AUTO_RSP},
     {ESP_UUID_LEN_16, (uint8_t *) &char_decl_uuid, ESP_GATT_PERM_READ, 1, 1,
      (uint8_t *) &char_prop_read_write}},
    {{ESP_GATT_RSP_BY_APP},
     {ESP_UUID_LEN_128, (uint8_t *) mos_rpc_data_uuid.uuid.uuid128,
      ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, 0, 0, NULL}},

    /* rx_ctl */
    {{ESP_GATT_AUTO_RSP},
     {ESP_UUID_LEN_16, (uint8_t *) &char_decl_uuid, ESP_GATT_PERM_READ, 1, 1,
      (uint8_t *) &char_prop_read_notify}},
    {{ESP_GATT_RSP_BY_APP},
     {ESP_UUID_LEN_128, (uint8_t *) mos_rpc_rx_ctl_uuid.uuid.uuid128,
      ESP_GATT_PERM_READ, 0, 0, NULL}},

    /* tx_ctl */
    {{ESP_GATT_AUTO_RSP},
     {ESP_UUID_LEN_16, (uint8_t *) &char_decl_uuid, ESP_GATT_PERM_READ, 1, 1,
      (uint8_t *) &char_prop_write}},
    {{ESP_GATT_RSP_BY_APP},
     {ESP_UUID_LEN_128, (uint8_t *) mos_rpc_tx_ctl_uuid.uuid.uuid128,
      ESP_GATT_PERM_WRITE, 0, 0, NULL}},
};

struct mg_rpc_gatts_ch_data {
  struct esp32_bt_session *bs;
  uint16_t send_offset;
  struct mg_str sending_frame;
  uint16_t expected_flen;
  struct mg_str receiving_frame;
  struct mg_rpc_channel *ch;
};

struct mg_rpc_gatts_rx_ctl_read_resp {
  uint32_t frame_len;
};

struct mg_rpc_gatts_rx_ctl_notify_data {
  uint32_t frame_len;
};

struct mg_rpc_gatts_tx_ctl_write_req {
  uint32_t frame_len;
};

static void mg_rpc_ch_gatts_ch_connect(struct mg_rpc_channel *ch) {
  (void) ch;
}

static bool mg_rpc_ch_gatts_send_frame(struct mg_rpc_channel *ch,
                                       const struct mg_str f) {
  bool ret = false;
  struct mg_rpc_gatts_ch_data *chd =
      (struct mg_rpc_gatts_ch_data *) ch->channel_data;
  mgos_lock();
  if (chd->bs == NULL) goto out;
  chd->sending_frame = mg_strdup(f);
  if (chd->sending_frame.p == NULL) goto out;
  chd->send_offset = 0;
  struct mg_rpc_gatts_rx_ctl_notify_data nd = {
      .frame_len = htonl(f.len),
  };
  esp_ble_gatts_send_indicate(chd->bs->bc->gatt_if, chd->bs->bc->conn_id,
                              mos_rpc_rx_ctl_ah, sizeof(nd), (uint8_t *) &nd,
                              false /* need_confirm */);
  ret = true;
out:
  mgos_unlock();
  return ret;
}

static void mg_rpc_ch_gatts_ch_close(struct mg_rpc_channel *ch) {
  struct mg_rpc_gatts_ch_data *chd =
      (struct mg_rpc_gatts_ch_data *) ch->channel_data;
  if (chd->sending_frame.len > 0) {
    ch->ev_handler(ch, MG_RPC_CHANNEL_FRAME_SENT, NULL);
  }
  if (chd->bs != NULL) {
    chd->bs->user_data = NULL;
  }
  ch->ev_handler(ch, MG_RPC_CHANNEL_CLOSED, NULL);
}

static void mg_rpc_ch_gatts_ch_destroy(struct mg_rpc_channel *ch) {
  free(ch->channel_data);
  free(ch);
}

static const char *mg_rpc_ch_gatts_get_type(struct mg_rpc_channel *ch) {
  (void) ch;
  return "GATTS";
}

static bool mg_rpc_channel_gatts_get_authn_info(struct mg_rpc_channel *ch,
                                                struct mg_rpc_authn *authn) {
  (void) ch;
  (void) authn;

  return false;
}

static bool mg_rpc_ch_gatts_is_persistent(struct mg_rpc_channel *ch) {
  (void) ch;
  return false;
}

static char *mg_rpc_ch_gatts_get_info(struct mg_rpc_channel *ch) {
  struct mg_rpc_gatts_ch_data *chd =
      (struct mg_rpc_gatts_ch_data *) ch->channel_data;
  char abuf[BT_ADDR_STR_LEN];
  char *s = NULL;
  if (chd->bs != NULL) {
    asprintf(&s, "if %d conn %d peer %s", chd->bs->bc->gatt_if,
             chd->bs->bc->conn_id,
             mgos_bt_addr_to_str(chd->bs->bc->peer_addr, abuf));
  }
  return s;
}

struct mg_rpc_channel *mg_rpc_ch_gatt(struct esp32_bt_session *bs) {
  struct mg_rpc_channel *ch = (struct mg_rpc_channel *) calloc(1, sizeof(*ch));
  ch->ch_connect = mg_rpc_ch_gatts_ch_connect;
  ch->send_frame = mg_rpc_ch_gatts_send_frame;
  ch->ch_close = mg_rpc_ch_gatts_ch_close;
  ch->ch_destroy = mg_rpc_ch_gatts_ch_destroy;
  ch->get_type = mg_rpc_ch_gatts_get_type;
  ch->is_persistent = mg_rpc_ch_gatts_is_persistent;
  ch->get_authn_info = mg_rpc_channel_gatts_get_authn_info;
  ch->get_info = mg_rpc_ch_gatts_get_info;
  struct mg_rpc_gatts_ch_data *chd =
      (struct mg_rpc_gatts_ch_data *) calloc(1, sizeof(*chd));
  chd->bs = bs;
  chd->ch = ch;
  ch->channel_data = chd;
  return ch;
}

void chan_add(void *arg) {
  struct mg_rpc_channel *ch = (struct mg_rpc_channel *) arg;
  mg_rpc_add_channel(mgos_rpc_get_global(), mg_mk_str(""), ch,
                     mgos_sys_config_get_rpc_gatts_is_trusted());
  ch->ev_handler(ch, MG_RPC_CHANNEL_OPEN, NULL);
}

static void frame_sent(void *arg) {
  struct mg_rpc_gatts_ch_data *chd = (struct mg_rpc_gatts_ch_data *) arg;
  free((void *) chd->sending_frame.p);
  chd->send_offset = 0;
  chd->sending_frame.len = 0;
  chd->sending_frame.p = NULL;
  chd->ch->ev_handler(chd->ch, MG_RPC_CHANNEL_FRAME_SENT, (void *) 1);
}

static bool mgos_rpc_ch_gatts_read_rx_ctl(struct mg_rpc_gatts_ch_data *chd,
                                          uint32_t trans_id) {
  chd->send_offset = 0;
  LOG(LL_DEBUG, ("%p flen %d", chd->ch, chd->sending_frame.len));
  struct mg_rpc_gatts_rx_ctl_read_resp rr = {
      .frame_len = htonl(chd->sending_frame.len),
  };
  esp_gatt_rsp_t rsp = {.attr_value = {.handle = mos_rpc_rx_ctl_ah,
                                       .offset = 0,
                                       .len = sizeof(rr)}};
  memcpy(rsp.attr_value.value, &rr, sizeof(rr));
  esp_ble_gatts_send_response(chd->bs->bc->gatt_if, chd->bs->bc->conn_id,
                              trans_id, ESP_GATT_OK, &rsp);
  return true;
}

static bool mgos_rpc_ch_gatts_read_data(struct mg_rpc_gatts_ch_data *chd,
                                        uint32_t trans_id) {
  mgos_lock();
  size_t len = chd->sending_frame.len - chd->send_offset;
  if (len > ESP_GATT_MAX_ATTR_LEN) len = ESP_GATT_MAX_ATTR_LEN;
  if (len > chd->bs->bc->mtu) len = chd->bs->bc->mtu - 1;
  LOG(LL_DEBUG, ("%p sending %d @ %d", chd->ch, len, chd->send_offset));
  esp_gatt_rsp_t rsp = {
      .attr_value = {.handle = mos_rpc_data_ah, .offset = 0, .len = len}};
  memcpy(rsp.attr_value.value, chd->sending_frame.p + chd->send_offset, len);
  mgos_unlock();

  esp_err_t r = esp_ble_gatts_send_response(
      chd->bs->bc->gatt_if, chd->bs->bc->conn_id, trans_id, ESP_GATT_OK, &rsp);
  if (r == ESP_OK) {
    if (len > 0) {
      chd->send_offset += len;
      if (chd->send_offset == chd->sending_frame.len) {
        mgos_invoke_cb(frame_sent, chd, false /* from_isr */);
      }
    }
  } else {
    LOG(LL_ERROR, ("esp_ble_gatts_send_response failed: %d", r));
    return false;
  }
  return true;
}

static bool mgos_rpc_ch_gatts_write_tx_ctl(struct mg_rpc_gatts_ch_data *chd,
                                           size_t len, const void *data) {
  struct mg_rpc_gatts_tx_ctl_write_req req;
  if (len < sizeof(req)) return false;
  memcpy(&req, data, sizeof(req));
  size_t flen = ntohl(req.frame_len);
  if (flen > MGOS_RPC_CHANNEL_GATTS_MAX_FRAME_LEN) {
    LOG(LL_ERROR, ("Incoming frame is too big: %u", flen));
    return false;
  }
  bool ret = true;
  chd->receiving_frame.len = 0;
  if (flen != chd->expected_flen) {
    chd->receiving_frame.p =
        (char *) realloc((void *) chd->receiving_frame.p, flen);
    if (chd->receiving_frame.p == NULL) ret = false;
  }
  chd->expected_flen = flen;
  LOG(LL_DEBUG,
      ("%p expected_flen %u ret %d", chd->ch, chd->expected_flen, ret));
  return ret;
}

static void frame_recd(void *arg) {
  struct mg_rpc_gatts_ch_data *chd = (struct mg_rpc_gatts_ch_data *) arg;
  free((void *) chd->sending_frame.p);
  chd->send_offset = 0;
  chd->sending_frame.len = 0;
  chd->sending_frame.p = NULL;
  chd->ch->ev_handler(chd->ch, MG_RPC_CHANNEL_FRAME_RECD,
                      &chd->receiving_frame);
  free((void *) chd->receiving_frame.p);
  chd->receiving_frame.p = NULL;
  chd->receiving_frame.len = 0;
  chd->expected_flen = 0;
}

static bool mgos_rpc_ch_gatts_write_data(struct mg_rpc_gatts_ch_data *chd,
                                         size_t len, const void *data) {
  if (chd->receiving_frame.len + len > chd->expected_flen) {
    LOG(LL_ERROR, ("%p unexpected frame data: %u + %u > %u", chd->ch,
                   chd->receiving_frame.len, len, chd->expected_flen));
    return false;
  }
  memcpy((void *) (chd->receiving_frame.p + chd->receiving_frame.len), data,
         len);
  chd->receiving_frame.len += len;
  LOG(LL_DEBUG, ("%p got %u of %u", chd->ch, chd->receiving_frame.len,
                 chd->expected_flen));
  if (chd->receiving_frame.len == chd->expected_flen) {
    mgos_invoke_cb(frame_recd, chd, false /* from_isr */);
  }
  return true;
}

static bool mgos_rpc_ch_gatts_ev(struct esp32_bt_session *bs,
                                 esp_gatts_cb_event_t ev,
                                 esp_ble_gatts_cb_param_t *ep) {
  bool ret = false;
  struct mg_rpc_channel *ch = NULL;
  struct mg_rpc_gatts_ch_data *chd = NULL;
  if (bs != NULL) { /* CREAT_ATTR_TAB is not associated with any session. */
    ch = (struct mg_rpc_channel *) bs->user_data;
    if (ch != NULL) {
      chd = (struct mg_rpc_gatts_ch_data *) ch->channel_data;
    }
  }
  switch (ev) {
    case ESP_GATTS_CREAT_ATTR_TAB_EVT: {
      const struct gatts_add_attr_tab_evt_param *p = &ep->add_attr_tab;
      uint16_t svch = p->handles[0];
      mos_rpc_data_ah = p->handles[2];
      mos_rpc_rx_ctl_ah = p->handles[4];
      mos_rpc_tx_ctl_ah = p->handles[6];
      LOG(LL_DEBUG,
          ("svch = %d data_ah = %d rx_ctl_ah = %d tx_ctl_ah = %d", svch,
           mos_rpc_data_ah, mos_rpc_rx_ctl_ah, mos_rpc_tx_ctl_ah));
      break;
    }
    case ESP_GATTS_CONNECT_EVT: {
      ch = mg_rpc_ch_gatt(bs);
      if (ch == NULL) break;
      bs->user_data = ch;
      mgos_invoke_cb(chan_add, ch, false /* from_isr */);
      break;
    }
    case ESP_GATTS_READ_EVT: {
      const struct gatts_read_evt_param *p = &ep->read;
      if (chd == NULL) break;
      if (p->handle == mos_rpc_data_ah) {
        ret = mgos_rpc_ch_gatts_read_data(chd, p->trans_id);
      } else if (p->handle == mos_rpc_rx_ctl_ah) {
        ret = mgos_rpc_ch_gatts_read_rx_ctl(chd, p->trans_id);
      }
      break;
    }
    case ESP_GATTS_WRITE_EVT: {
      const struct gatts_write_evt_param *p = &ep->write;
      if (chd == NULL) break;
      if (p->handle == mos_rpc_data_ah) {
        ret = mgos_rpc_ch_gatts_write_data(chd, p->len, p->value);
      } else if (p->handle == mos_rpc_tx_ctl_ah) {
        ret = mgos_rpc_ch_gatts_write_tx_ctl(chd, p->len, p->value);
      }
      break;
    }
    case ESP_GATTS_DISCONNECT_EVT: {
      if (ch == NULL) break;
      mgos_invoke_cb((void (*) (void *)) mg_rpc_ch_gatts_ch_close, ch,
                     false /* from_isr */);
      break;
    }
    default:
      break;
  }
  return ret;
}

bool mgos_rpc_gatts_init(void) {
  if (mgos_rpc_get_global() == NULL) return true;
  mgos_bt_gatts_register_service(
      mos_rpc_gatt_db, sizeof(mos_rpc_gatt_db) / sizeof(mos_rpc_gatt_db[0]),
      mgos_rpc_ch_gatts_ev);
  return true;
}
