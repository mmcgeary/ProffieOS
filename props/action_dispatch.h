// ProffieOS/props/action_dispatch.h
#ifndef PROPS_ACTION_DISPATCH_H
#define PROPS_ACTION_DISPATCH_H

#include "runtime_config.h"
#include "button_profiles.h"
#include "../common/events.h"

int ResolveButtonSlot(uint32_t button, uint32_t event, uint32_t modifiers, uint8_t num_buttons) {
  // Combo detection (modifier held while another button acts)
  if (num_buttons >= 2) {
    if ((modifiers & BUTTON_POWER) && button == BUTTON_AUX && event == EVENT_HELD) {
      return SLOT_PWR_AUX_HOLD;
    }
    if ((modifiers & BUTTON_AUX) && button == BUTTON_POWER && event == EVENT_HELD) {
      return SLOT_AUX_PWR_HOLD;
    }
    if ((modifiers & BUTTON_AUX) && button == BUTTON_POWER && event == EVENT_CLICK_SHORT) {
      return SLOT_PWR_AUX_CLICK;
    }
  }

  if (num_buttons >= 3) {
    if ((modifiers & BUTTON_POWER) && button == BUTTON_AUX2 && event == EVENT_HELD) {
      return SLOT_PWR_AUX2_HOLD;
    }
    if ((modifiers & BUTTON_AUX2) && button == BUTTON_POWER && event == EVENT_HELD) {
      return SLOT_AUX2_PWR_HOLD;
    }
    if ((modifiers & BUTTON_AUX) && button == BUTTON_AUX2 && event == EVENT_HELD) {
      return SLOT_AUX_AUX2_HOLD;
    }
    if ((modifiers & BUTTON_AUX2) && button == BUTTON_AUX && event == EVENT_HELD) {
      return SLOT_AUX2_AUX_HOLD;
    }
    if ((modifiers & (BUTTON_POWER | BUTTON_AUX | BUTTON_AUX2)) ==
        (BUTTON_POWER | BUTTON_AUX | BUTTON_AUX2)) {
      return SLOT_ALL_HOLD;
    }
  }

  // Single button events
  if (button == BUTTON_POWER) {
    switch (event) {
      case EVENT_CLICK_SHORT:         return SLOT_PWR_CLICK;
      case EVENT_CLICK_LONG:          return SLOT_PWR_LONG_CLICK;
      case EVENT_HELD:                return SLOT_PWR_HOLD;
      case EVENT_HELD_LONG:           return SLOT_PWR_HOLD_LONG;
      case EVENT_SECOND_CLICK_SHORT:  return SLOT_PWR_DOUBLE_CLICK;
      default: break;
    }
  }

  if (button == BUTTON_AUX && num_buttons >= 2) {
    switch (event) {
      case EVENT_CLICK_SHORT:         return SLOT_AUX_CLICK;
      case EVENT_CLICK_LONG:          return SLOT_AUX_LONG_CLICK;
      case EVENT_HELD:                return SLOT_AUX_HOLD;
      case EVENT_HELD_LONG:           return SLOT_AUX_HOLD_LONG;
      case EVENT_SECOND_CLICK_SHORT:  return SLOT_AUX_DOUBLE_CLICK;
      default: break;
    }
  }

  if (button == BUTTON_AUX2 && num_buttons >= 3) {
    switch (event) {
      case EVENT_CLICK_SHORT:         return SLOT_AUX2_CLICK;
      case EVENT_CLICK_LONG:          return SLOT_AUX2_LONG_CLICK;
      case EVENT_HELD:                return SLOT_AUX2_HOLD;
      case EVENT_HELD_LONG:           return SLOT_AUX2_HOLD_LONG;
      case EVENT_SECOND_CLICK_SHORT:  return SLOT_AUX2_DOUBLE_CLICK;
      default: break;
    }
  }

  return -1;
}

template<class PROP>
void ExecuteAction(IniAction action, PROP* prop) {
  switch (action) {
    case ACTION_NONE:
      break;
    case ACTION_ON:
      prop->On();
      break;
    case ACTION_OFF:
      prop->Off();
      break;
    case ACTION_BLAST:
      SaberBase::DoBlast();
      break;
    case ACTION_CLASH:
      SaberBase::DoClash();
      break;
    case ACTION_LOCKUP:
      if (SaberBase::Lockup() == SaberBase::LOCKUP_NONE) {
        SaberBase::SetLockup(SaberBase::LOCKUP_NORMAL);
        SaberBase::DoBeginLockup();
      }
      break;
    case ACTION_DRAG:
      if (SaberBase::Lockup() == SaberBase::LOCKUP_NONE) {
        SaberBase::SetLockup(SaberBase::LOCKUP_DRAG);
        SaberBase::DoBeginLockup();
      }
      break;
    case ACTION_MELT:
      if (SaberBase::Lockup() == SaberBase::LOCKUP_NONE) {
        SaberBase::SetLockup(SaberBase::LOCKUP_MELT);
        SaberBase::DoBeginLockup();
      }
      break;
    case ACTION_LIGHTNING_BLOCK:
      if (SaberBase::Lockup() == SaberBase::LOCKUP_NONE) {
        SaberBase::SetLockup(SaberBase::LOCKUP_LIGHTNING_BLOCK);
        SaberBase::DoBeginLockup();
      }
      break;
    case ACTION_FORCE:
      SaberBase::DoForce();
      break;
    case ACTION_STAB:
      SaberBase::DoStab();
      break;
    case ACTION_COLOR_CHANGE:
      prop->ToggleColorChangeMode();
      break;
    case ACTION_NEXT_PRESET:
      prop->next_preset();
      break;
    case ACTION_PREV_PRESET:
      prop->previous_preset();
      break;
    case ACTION_VOLUME_UP:
      prop->VolumeUp();
      break;
    case ACTION_VOLUME_DOWN:
      prop->VolumeDown();
      break;
    case ACTION_TRACK_PLAYER:
      prop->StartOrStopTrack();
      break;
    case ACTION_BATTERY_LEVEL:
      prop->SayBatteryLevel();
      break;
    case ACTION_QUOTE:
      prop->PlayQuote();
      break;
    case ACTION_ENTER_COLOR_CHANGE:
      prop->ToggleColorChangeMode();
      break;
    case ACTION_EXIT_COLOR_CHANGE:
      prop->ToggleColorChangeMode();
      break;
    default:
      break;
  }
}

bool IsSustainedAction(IniAction action) {
  return action == ACTION_LOCKUP ||
         action == ACTION_DRAG ||
         action == ACTION_MELT ||
         action == ACTION_LIGHTNING_BLOCK;
}

#endif // PROPS_ACTION_DISPATCH_H
