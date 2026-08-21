#include "control_ack.h"

control_ack_t control_ack_decide(bool received, uint32_t awaited_seq,
                                 const control_apply_result_t *result,
                                 bool rx_fault_pending)
{
  if (!received)
  {
    return CONTROL_ACK_ERR_TIMEOUT;
  }

  /* mailbox overwrite나 순서 뒤바뀜으로 다른 명령의 결과가 왔다. 이 seq의 duty가
     반영됐다는 근거가 없으므로 ok를 만들 수 없다. */
  if (result->seq != awaited_seq)
  {
    return CONTROL_ACK_ERR_SEQ;
  }

  if (result->kind != CONTROL_APPLY_OK)
  {
    return CONTROL_ACK_ERR_REJECTED;
  }

  /* 적용은 됐지만 그 직후 RX 손상이 출력을 다시 0으로 만들었다. 지금 CCR에 없는
     duty를 ok로 보고하지 않는다. */
  if (rx_fault_pending)
  {
    return CONTROL_ACK_ERR_RX_FAULT;
  }

  return CONTROL_ACK_OK;
}
