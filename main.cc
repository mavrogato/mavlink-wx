
#include <iostream>
#include <numeric>
#include <ranges>

#include <wx/wxprec.h>
#ifndef WX_PRECOMP
# include <wx/wx.h>
#endif
#include <wx/splitter.h>
#include <wx/listctrl.h>
#include <wx/textctrl.h>
#include <wx/propgrid/manager.h>
#include <wx/propgrid/propgrid.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"
#define MAVLINK_EXTERNAL_RX_STATUS // No m_mavlink_status array defined in function
# include <mavlink_types.h>
inline mavlink_status_t m_mavlink_status[MAVLINK_COMM_NUM_BUFFERS] = { };
# include <standard/mavlink.h>
#pragma GCC diagnostic pop

constexpr mavlink_message_info_t meta_messages[] = MAVLINK_MESSAGE_INFO;
constexpr mavlink_msg_entry_t meta_messages_extra[] = MAVLINK_MESSAGE_CRCS;

constexpr auto meta_fields(mavlink_message_info_t const& meta_message) noexcept {
    auto bgn = std::ranges::begin(meta_message.fields);
    return std::ranges::subrange(bgn, bgn + meta_message.num_fields);
}

template <mavlink_message_type_t TYPE> struct type_of_t;
template <> struct type_of_t<MAVLINK_TYPE_CHAR    > { using type = char;     static constexpr auto name = "char";     };
template <> struct type_of_t<MAVLINK_TYPE_UINT8_T > { using type = uint8_t;  static constexpr auto name = "uint8_t";  };
template <> struct type_of_t<MAVLINK_TYPE_INT8_T  > { using type = int8_t;   static constexpr auto name = "int8_t";   };
template <> struct type_of_t<MAVLINK_TYPE_UINT16_T> { using type = uint16_t; static constexpr auto name = "uint16_t"; };
template <> struct type_of_t<MAVLINK_TYPE_INT16_T > { using type = int16_t;  static constexpr auto name = "int16_t";  };
template <> struct type_of_t<MAVLINK_TYPE_UINT32_T> { using type = uint32_t; static constexpr auto name = "uint32_t"; };
template <> struct type_of_t<MAVLINK_TYPE_INT32_T > { using type = int32_t;  static constexpr auto name = "int32_t";  };
template <> struct type_of_t<MAVLINK_TYPE_UINT64_T> { using type = uint64_t; static constexpr auto name = "uint64_t"; };
template <> struct type_of_t<MAVLINK_TYPE_INT64_T > { using type = int64_t;  static constexpr auto name = "int64_t";  };
template <> struct type_of_t<MAVLINK_TYPE_FLOAT   > { using type = float;    static constexpr auto name = "float";    };
template <> struct type_of_t<MAVLINK_TYPE_DOUBLE  > { using type = double;   static constexpr auto name = "double";   };
template <mavlink_message_type_t TYPE> using type_of = type_of_t<TYPE>::type;
template <mavlink_message_type_t TYPE> constexpr auto type_name_of = type_of_t<TYPE>::name;
template <mavlink_message_type_t TYPE> constexpr auto minimum = std::numeric_limits<type_of<TYPE>>::min();
template <mavlink_message_type_t TYPE> constexpr auto maximum = std::numeric_limits<type_of<TYPE>>::max();

constexpr size_t bytes(mavlink_message_type_t type) noexcept {
    switch (type) {
    case MAVLINK_TYPE_CHAR    : return sizeof (type_of<MAVLINK_TYPE_CHAR    >);
    case MAVLINK_TYPE_UINT8_T : return sizeof (type_of<MAVLINK_TYPE_UINT8_T >);
    case MAVLINK_TYPE_INT8_T  : return sizeof (type_of<MAVLINK_TYPE_INT8_T  >);
    case MAVLINK_TYPE_UINT16_T: return sizeof (type_of<MAVLINK_TYPE_UINT16_T>);
    case MAVLINK_TYPE_INT16_T : return sizeof (type_of<MAVLINK_TYPE_INT16_T >);
    case MAVLINK_TYPE_UINT32_T: return sizeof (type_of<MAVLINK_TYPE_UINT32_T>);
    case MAVLINK_TYPE_INT32_T : return sizeof (type_of<MAVLINK_TYPE_INT32_T >);
    case MAVLINK_TYPE_UINT64_T: return sizeof (type_of<MAVLINK_TYPE_UINT64_T>);
    case MAVLINK_TYPE_INT64_T : return sizeof (type_of<MAVLINK_TYPE_INT64_T >);
    case MAVLINK_TYPE_FLOAT   : return sizeof (type_of<MAVLINK_TYPE_FLOAT   >);
    case MAVLINK_TYPE_DOUBLE  : return sizeof (type_of<MAVLINK_TYPE_DOUBLE  >);
    default                   : return 0;
    }
}

constexpr char const* type_name(mavlink_message_type_t type) noexcept {
    switch (type) {
    case MAVLINK_TYPE_CHAR    : return type_name_of<MAVLINK_TYPE_CHAR    >;
    case MAVLINK_TYPE_UINT8_T : return type_name_of<MAVLINK_TYPE_UINT8_T >;
    case MAVLINK_TYPE_INT8_T  : return type_name_of<MAVLINK_TYPE_INT8_T  >;
    case MAVLINK_TYPE_UINT16_T: return type_name_of<MAVLINK_TYPE_UINT16_T>;
    case MAVLINK_TYPE_INT16_T : return type_name_of<MAVLINK_TYPE_INT16_T >;
    case MAVLINK_TYPE_UINT32_T: return type_name_of<MAVLINK_TYPE_UINT32_T>;
    case MAVLINK_TYPE_INT32_T : return type_name_of<MAVLINK_TYPE_INT32_T >;
    case MAVLINK_TYPE_UINT64_T: return type_name_of<MAVLINK_TYPE_UINT64_T>;
    case MAVLINK_TYPE_INT64_T : return type_name_of<MAVLINK_TYPE_INT64_T >;
    case MAVLINK_TYPE_FLOAT   : return type_name_of<MAVLINK_TYPE_FLOAT   >;
    case MAVLINK_TYPE_DOUBLE  : return type_name_of<MAVLINK_TYPE_DOUBLE  >;
    default                   : return "unknown";
    }
}

constexpr size_t bytes(mavlink_field_info_t const& meta_field) noexcept {
    size_t ret = bytes(meta_field.type);
    if (auto len = meta_field.array_length) ret *= len;
    return ret;
}

constexpr size_t bytes(mavlink_message_info_t const& meta_message) noexcept {
    return std::accumulate(meta_fields(meta_message).begin(),
                           meta_fields(meta_message).end(),
                           size_t(0),
                           [](size_t lhs, mavlink_field_info_t const& rhs) noexcept {
                               return lhs + bytes(rhs);
                           });
}

class AppEntry : public wxApp {
public:
    virtual bool OnInit() {
        auto frame = new wxFrame(nullptr, wxID_ANY, "default caption");
        {
            auto menuFile = new wxMenu();
            menuFile->Append(wxID_EXIT);
            auto menuBar = new wxMenuBar();
            menuBar->Append(menuFile, "&File");
            frame->SetMenuBar(menuBar);
            Bind(wxEVT_MENU, [frame](auto) { frame->Close(); }, wxID_EXIT);
        }
        wxPanel* panel_list = nullptr;
        wxPanel* panel_grid = nullptr;
        {
            auto split_horz = new wxSplitterWindow(frame);
            split_horz->SetMinimumPaneSize(50);
            panel_list = new wxPanel(split_horz);
            panel_grid = new wxPanel(split_horz);
            split_horz->SplitHorizontally(panel_list, panel_grid, 296);
        }
        /////////////////////////////////////////////////////////////////////////////
        {
            auto list = new wxListCtrl(panel_list,
                                       wxID_ANY,
                                       wxDefaultPosition,
                                       wxDefaultSize,
                                       wxLC_REPORT | wxLC_SINGLE_SEL);
            list->InsertColumn(0, "message name", wxLIST_FORMAT_LEFT, 296);
            list->InsertColumn(-1, "msgid", wxLIST_FORMAT_RIGHT, 64);
            list->InsertColumn(-1, "fields", wxLIST_FORMAT_RIGHT, 64);
            list->InsertColumn(-1, "bytes", wxLIST_FORMAT_RIGHT, 64);
            for (size_t i = 0; i < std::size(meta_messages); i++) {
                auto const& msg = meta_messages[i];
                wxListItem item;
                item.SetId(i);
                item.SetText(msg.name);
                list->InsertItem(item);
                list->SetItem(i, 1, std::to_string(msg.msgid));
                list->SetItem(i, 2, std::to_string(msg.num_fields));
                list->SetItem(i, 3, std::to_string(bytes(msg)));
            }
            /////////////////////////////////////////////////////////////////////////////
            {
                auto sizer_list = new wxBoxSizer(wxHORIZONTAL);
                sizer_list->Add(list, 1, wxEXPAND, 50);
                panel_list->SetSizer(sizer_list);
            }
            /////////////////////////////////////////////////////////////////////////////
            auto pmgr = new wxPropertyGridManager(panel_grid,
                                                  wxID_ANY,
                                                  wxDefaultPosition,
                                                  wxDefaultSize,
                                                  wxPG_BOLD_MODIFIED |
                                                  wxPG_SPLITTER_AUTO_CENTER |
                                                  wxPG_TOOLBAR |
                                                  wxPG_DESCRIPTION);
            pmgr->SetExtraStyle(wxPG_EX_MODE_BUTTONS |
                                wxPG_EX_NATIVE_DOUBLE_BUFFERING |
                                wxPG_EX_MULTIPLE_SELECTION);
            pmgr->SetDescBoxHeight(48);
            /////////////////////////////////////////////////////////////////////////////
            Bind(wxEVT_LIST_ITEM_SELECTED, [list, grid = pmgr->GetGrid()](auto args) {
                grid->Clear();
                //grid->Append(new wxPropertyCategory("Header"));
                grid->Append(new wxPropertyCategory("Payload"));
                for (auto const& fld : meta_fields(meta_messages[args.GetIndex()])) {
                    if (MAVLINK_TYPE_CHAR == fld.type) {
                        grid->Append(new wxStringProperty(fld.name));
                    }
                    else if (0 == fld.array_length) {
                        switch (fld.type) {
                        case MAVLINK_TYPE_UINT8_T : {
                            grid->Append(new wxUIntProperty(fld.name));
                            grid->SetPropertyAttribute(fld.name, wxPG_ATTR_MIN, minimum<MAVLINK_TYPE_UINT8_T>);
                            grid->SetPropertyAttribute(fld.name, wxPG_ATTR_MAX, maximum<MAVLINK_TYPE_UINT8_T>);
                            grid->SetPropertyAttribute(fld.name, wxPG_UINT_PREFIX, wxPG_PREFIX_0x);
                            grid->SetPropertyAttribute(fld.name, wxPG_UINT_BASE, wxPG_BASE_HEX);
                            break;
                        }
                        case MAVLINK_TYPE_INT8_T  : {
                            grid->Append(new wxIntProperty(fld.name));
                            grid->SetPropertyAttribute(fld.name, wxPG_ATTR_MIN, minimum<MAVLINK_TYPE_INT8_T>);
                            grid->SetPropertyAttribute(fld.name, wxPG_ATTR_MAX, maximum<MAVLINK_TYPE_INT8_T>);
                            break;
                        }
                        case MAVLINK_TYPE_UINT16_T: {
                            grid->Append(new wxUIntProperty(fld.name));
                            grid->SetPropertyAttribute(fld.name, wxPG_ATTR_MIN, minimum<MAVLINK_TYPE_UINT16_T>);
                            grid->SetPropertyAttribute(fld.name, wxPG_ATTR_MAX, maximum<MAVLINK_TYPE_UINT16_T>);
                            grid->SetPropertyAttribute(fld.name, wxPG_UINT_PREFIX, wxPG_PREFIX_0x);
                            grid->SetPropertyAttribute(fld.name, wxPG_UINT_BASE, wxPG_BASE_HEX);
                            break;
                        }
                        case MAVLINK_TYPE_INT16_T : {
                            grid->Append(new wxIntProperty(fld.name));
                            grid->SetPropertyAttribute(fld.name, wxPG_ATTR_MIN, minimum<MAVLINK_TYPE_INT16_T>);
                            grid->SetPropertyAttribute(fld.name, wxPG_ATTR_MAX, maximum<MAVLINK_TYPE_INT16_T>);
                            break;
                        }
                        case MAVLINK_TYPE_UINT32_T: {
                            grid->Append(new wxUIntProperty(fld.name));
                            grid->SetPropertyAttribute(fld.name, wxPG_ATTR_MIN, (int64_t) minimum<MAVLINK_TYPE_UINT32_T>);
                            grid->SetPropertyAttribute(fld.name, wxPG_ATTR_MAX, (int64_t) maximum<MAVLINK_TYPE_UINT32_T>);
                            grid->SetPropertyAttribute(fld.name, wxPG_UINT_PREFIX, wxPG_PREFIX_0x);
                            grid->SetPropertyAttribute(fld.name, wxPG_UINT_BASE, wxPG_BASE_HEX);
                            break;
                        }
                        case MAVLINK_TYPE_INT32_T : {
                            grid->Append(new wxIntProperty(fld.name));
                            grid->SetPropertyAttribute(fld.name, wxPG_ATTR_MIN, minimum<MAVLINK_TYPE_INT32_T>);
                            grid->SetPropertyAttribute(fld.name, wxPG_ATTR_MAX, maximum<MAVLINK_TYPE_INT32_T>);
                            break;
                        }
                        case MAVLINK_TYPE_UINT64_T: {
                            grid->Append(new wxUIntProperty(fld.name));
                            grid->SetPropertyAttribute(fld.name, wxPG_ATTR_MIN, (int64_t) minimum<MAVLINK_TYPE_UINT64_T>);
                            //grid->SetPropertyAttribute(fld.name, wxPG_ATTR_MAX, maximum<MAVLINK_TYPE_UINT64_T>);
                            grid->SetPropertyAttribute(fld.name, wxPG_UINT_PREFIX, wxPG_PREFIX_0x);
                            grid->SetPropertyAttribute(fld.name, wxPG_UINT_BASE, wxPG_BASE_HEX);
                            break;
                        }
                        case MAVLINK_TYPE_INT64_T :
                            grid->Append(new wxIntProperty(fld.name));
                            grid->SetPropertyAttribute(fld.name, wxPG_ATTR_MIN, minimum<MAVLINK_TYPE_INT64_T>);
                            grid->SetPropertyAttribute(fld.name, wxPG_ATTR_MAX, maximum<MAVLINK_TYPE_INT64_T>);
                            break;
                        case MAVLINK_TYPE_FLOAT   :
                        case MAVLINK_TYPE_DOUBLE  :
                            grid->Append(new wxFloatProperty(fld.name));
                            break;
                        case MAVLINK_TYPE_CHAR:
                        default:
                            break;
                        }
                    }
                    else {
                        grid->Append(new wxArrayStringProperty(fld.name, wxPG_LABEL));
                    }
                    std::string type = type_name(fld.type);
                    if (fld.array_length) {
                        type += "[" + std::to_string(fld.array_length) + "]";
                    }
                    grid->SetPropertyHelpString(fld.name, type);
                }
            }, wxID_ANY);
            /////////////////////////////////////////////////////////////////////////////
            Bind(wxEVT_PG_CHANGED, [grid = pmgr->GetGrid()](auto args) {
                std::cout << "done" << std::endl;
            }, wxID_ANY);
            /////////////////////////////////////////////////////////////////////////////
            {
                list->SetItemState(0, wxLIST_STATE_SELECTED, wxLIST_STATE_SELECTED);
                auto sizer_grid = new wxBoxSizer(wxHORIZONTAL);
                sizer_grid->Add(pmgr, 1, wxEXPAND, 50);
                panel_grid->SetSizer(sizer_grid);
            }
        }
        /////////////////////////////////////////////////////////////////////////////
        frame->SetSize({ 512, 832 });
        frame->Show(true);
        return true;
    }
};
IMPLEMENT_APP(AppEntry)
