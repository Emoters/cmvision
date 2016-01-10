/*********************************************************************
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/** \author Nate Koenig and Peter Pastor */

#include <string>
#include <cv_bridge/cv_bridge.h>

#include "color_gui.h"
#include "conversions.h"

#define TIMER_INTERVAL_UI           33
#define TIMER_INTERVAL_DISPLAY      int(3000.0/TIMER_INTERVAL_UI)

#define RGB2YUV(r, g, b, y, u, v)\
  y = (306*r + 601*g + 117*b)  >> 10;\
  u = ((-172*r - 340*g + 512*b) >> 10)  + 128;\
  v = ((512*r - 429*g - 83*b) >> 10) + 128;\
  y = y < 0 ? 0 : y;\
  u = u < 0 ? 0 : u;\
  v = v < 0 ? 0 : v;\
  y = y > 255 ? 255 : y;\
  u = u > 255 ? 255 : u;\
  v = v > 255 ? 255 : v


bool ColorGuiApp::OnInit()
{
  std::string image_topic, config_file;
  char **local_argv = new char*[ argc ];
  for (int i =0; i < argc; i++)
    local_argv[i] = strdup( wxString( argv[i] ).mb_str() );

  ros::init(argc, local_argv, "cmvision");
  ros::NodeHandle node_handle("~");
    
  // Subscribe to an image stream
  node_handle.param("image_topic", image_topic, std::string("stereo/left/image_rect_color"));
  node_handle.param("color_file", config_file, std::string(""));
  image_subscriber_ = node_handle.subscribe(image_topic, 1, &ColorGuiApp::imageCB, this);

  frame_ = new ColorGuiFrame(config_file.c_str());
  frame_->Show(true);
  SetTopWindow(frame_);

  update_timer_ = new wxTimer(this);
  update_timer_->Start( TIMER_INTERVAL_UI );

  Connect( update_timer_->GetId(), wxEVT_TIMER, wxTimerEventHandler( ColorGuiApp::OnUpdate ), NULL, this);

  return true;
}

void ColorGuiApp::OnUpdate( wxTimerEvent &event )
{
  ros::spinOnce();
  frame_->OnTimer();
}

void ColorGuiApp::imageCB(const sensor_msgs::ImageConstPtr& msg)
{
  frame_->DrawImage( msg );
}

ColorGuiFrame::ColorGuiFrame(const char *szConfigFile)
  : wxFrame(NULL, -1, wxT("Color Gui"), wxDefaultPosition, wxSize(800,600), wxDEFAULT_FRAME_STYLE),
    sConfigFile(szConfigFile),
    y_low_last(-1), y_high_last(-1), u_low_last(-1), u_high_last(-1), v_low_last(-1), v_high_last(-1)
{
  std::stringstream ssFormat;
  wxInitAllImageHandlers();
    
  sStatus = "Click to select colors. Scroll wheel zooms. [Ctrl-R:Reset,Ctrl-S:Save,Ctrl-Z:Undo,Ctrl-Q:Quit]";
  iStatusCounter = 0;

  wxMenuBar *menuBar = new wxMenuBar;
  wxMenu *file_menu = new wxMenu;

  wxMenuItem *item = file_menu->Append(ID_Reset, wxT("&Reset\tCtrl-R"));
  Connect(item->GetId(), wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler(ColorGuiFrame::OnReset), NULL, this);

  item = file_menu->Append(ID_Save, wxT("&Save\tCtrl-S"));
  Connect(item->GetId(), wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler(ColorGuiFrame::OnSave), NULL, this);
    
  item = file_menu->Append(ID_Undo, wxT("&Undo\tCtrl-Z"));
  Connect(item->GetId(), wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler(ColorGuiFrame::OnUndo), NULL, this);

  item = file_menu->Append(wxID_EXIT, wxT("&Quit\tCtrl-Q"));
  Connect(item->GetId(), wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler(ColorGuiFrame::OnQuit), NULL, this);
    
  menuBar->Append( file_menu, _("&File") );

  SetMenuBar( menuBar );

  CreateStatusBar();
  SetStatusText(wxString::FromAscii(sStatus.c_str()));
  ROS_INFO("%s", sStatus.c_str());

  image_panel_ = new wxPanel(this, wxID_ANY, wxPoint(0,0), wxSize(640,480));

  wxStaticText *rgblabel = new wxStaticText(this, -1, wxT("RGB:"));
  rgbText_ = new wxTextCtrl(this,-1,wxT(""));

  wxStaticText *yuvLabel = new wxStaticText(this, -1, wxT("YUV:"));
  yuvText_ = new wxTextCtrl(this,-1,wxT(""));
    
  wxStaticText *colorIdxLabel = new wxStaticText(this, -1, wxT("Index:"));
  colorIdx_ = new wxTextCtrl(this,-1,wxT("") );
    
  wxStaticText *colorLabel = new wxStaticText(this, -1, wxT("Color:"));
  colorCombo_ = new wxComboBox(this,-1,wxT(""), wxDefaultPosition, wxDefaultSize,
                                0, NULL, wxTE_PROCESS_ENTER);
  Connect(colorCombo_->GetId(), wxEVT_COMMAND_COMBOBOX_SELECTED,
          wxCommandEventHandler(ColorGuiFrame::OnColorChange), NULL, this);
  Connect(colorCombo_->GetId(), wxEVT_COMMAND_TEXT_ENTER,
          wxCommandEventHandler(ColorGuiFrame::OnColorEnter), NULL, this);
  //info: http://docs.wxwidgets.org/trunk/classwx_command_event.html, http://proton-ce.sourceforge.net/rc/wxwidgets/docs/html/wx/wx_wxcommandevent.html
    
  vision_ = new CMVision();
  if (!sConfigFile.empty()) {
      if (vision_->loadOptions(sConfigFile.c_str())) {
          colorCombo_->Clear();
          for (int c=0; c<vision_->numColors(); c++) {
              ssFormat.str(""); ssFormat.clear();
              ssFormat /*<< c  << " " */<< vision_->getColorName(c);
              colorCombo_->Append(wxString::FromAscii(ssFormat.str().c_str()));
          }
          ROS_INFO("Loaded '%s' with %d colors...", sConfigFile.c_str(), vision_->numColors());

          wxCommandEvent wxC;
          colorCombo_->SetSelection(0);
          OnColorChange(wxC);
      }
      sConfigFile += ".new";
  }
  wxBoxSizer *hsizer1 = new wxBoxSizer(wxHORIZONTAL);
  wxBoxSizer *hsizer2 = new wxBoxSizer(wxHORIZONTAL);
  wxBoxSizer *hsizer3 = new wxBoxSizer(wxHORIZONTAL);
  wxBoxSizer *vsizer = new wxBoxSizer(wxVERTICAL);

  hsizer1->Add( colorIdxLabel, 0, wxALIGN_CENTER_VERTICAL| wxLEFT, 10);
  hsizer1->Add( colorIdx_, 1, wxALIGN_CENTER_VERTICAL | wxLEFT, 2);
  hsizer1->Add( colorLabel, 0, wxALIGN_CENTER_VERTICAL| wxLEFT, 10);
  hsizer1->Add( colorCombo_, 1, wxALIGN_CENTER_VERTICAL | wxLEFT, 2);

  hsizer2->Add( rgblabel, 0, wxALIGN_CENTER_VERTICAL| wxLEFT, 10);
  hsizer2->Add( rgbText_, 1, wxALIGN_CENTER_VERTICAL | wxLEFT, 2);

  hsizer2->Add( yuvLabel, 0, wxALIGN_CENTER_VERTICAL | wxLEFT, 10);
  hsizer2->Add( yuvText_, 1, wxALIGN_CENTER_VERTICAL | wxLEFT, 2);
  hsizer2->Add(50,10,0);

  wxStaticText *padLabel = new wxStaticText(this, -1, wxT(""));
  hsizer3->Add( padLabel, 0, wxALIGN_CENTER_VERTICAL| wxLEFT, 10);
    
  vsizer->Add(image_panel_, 0, wxEXPAND);
  vsizer->Add(20,2,0);
  vsizer->Add(hsizer1, 0, wxALIGN_LEFT | wxEXPAND );
  vsizer->Add(20,2,0);
  vsizer->Add(hsizer2, 0, wxALIGN_LEFT | wxEXPAND );
  vsizer->Add(20,2,0);
  vsizer->Add(hsizer3, 0, wxALIGN_LEFT | wxEXPAND );
  this->SetSizer(vsizer);

	width_ = 0;
	height_ = 0;

  scale_ = 1.0;
  width_ = 0;
  height_ = 0;

	rgb_image_ = NULL;
  uyvy_image_ = NULL;

  image_panel_->Connect(wxEVT_LEFT_UP, wxMouseEventHandler(ColorGuiFrame::OnClick), NULL, this);

  Connect(wxEVT_MOUSEWHEEL, wxMouseEventHandler(ColorGuiFrame::OnMouseWheel));
}

void ColorGuiFrame::OnTimer( void )
{
    if (iStatusCounter) {
        iStatusCounter--;
    }
    else if (iStatusCounter==0) {
        iStatusCounter--;
        SetStatusText(wxString::FromAscii(sStatus.c_str()));
    }
}

void ColorGuiFrame::OnQuit(wxCommandEvent &event)
{
  Close(true);
}

void ColorGuiFrame::DrawImage(const sensor_msgs::ImageConstPtr& msg)
{
  IplImage cvImageRef, *cvImage;
	CvSize size;

	const sensor_msgs::Image img = *msg;

	// Get the image as and RGB image
        cv_bridge::CvImagePtr image_ptr = cv_bridge::toCvCopy(msg);
        cvImageRef = IplImage(image_ptr->image);
        cvImage = &cvImageRef;

	size = cvGetSize(cvImage);

  if (width_ != size.width || height_ != size.height)
  {
    if (!rgb_image_)
      delete[] rgb_image_;
    rgb_image_ = new unsigned char[size.width * size.height * 3];

    if (!uyvy_image_)
      delete[] uyvy_image_;
    uyvy_image_ = new unsigned char[size.width * size.height * 2];

    if (!(vision_->initialize(size.width, size.height)))
    {
      width_ = height_ = 0;
      ROS_ERROR("Vision init failed.");
      return;
    }
  }

  width_ = size.width;
  height_ = size.height;

  memcpy(rgb_image_, cvImage->imageData, width_ * height_ * 3);

  // Convert image to YUV color space
  rgb2uyvy(rgb_image_, uyvy_image_, width_ * height_);

	// Find the color blobs
	if (!vision_->processFrame(reinterpret_cast<image_pixel*> (uyvy_image_)))
	{
		ROS_ERROR("Frame error.");
		return;
	}

  int xsrc = (scale_pos_x_*scale_) - scale_pos_x_;
  int ysrc = (scale_pos_y_*scale_) - scale_pos_y_;

  wxImage image(width_, height_, rgb_image_, true);
  image.Rescale(width_*scale_,height_*scale_);

  wxBitmap bitmap(image);

  wxMemoryDC memDC;
  memDC.SelectObject(bitmap);

  wxClientDC dc(image_panel_);
  if (xsrc < 0 || ysrc < 0)
    dc.Clear();

  dc.Blit(0,0, 640, 480, &memDC, xsrc, ysrc);

	// Get all the blobs
	for (int ch = 0; ch < CMV_MAX_COLORS; ++ch)
	{
		// Get the descriptive color
		rgb c = vision_->getColorVisual(ch);

		// Grab the regions for this color
		CMVision::region* r = NULL;

		for (r = vision_->getRegions(ch); r != NULL; r = r->next)
		{
      dc.SetBrush(*wxTRANSPARENT_BRUSH);
      int x1 = (r->x1*scale_) - xsrc;
      int y1 = (r->y1*scale_) - ysrc;
      int x2 = (r->x2*scale_) - xsrc;
      int y2 = (r->y2*scale_) - ysrc;

      int w = x2 - x1;
      int h = y2 - y1;

      dc.DrawRectangle(x1, y1, w, h);
		}
	}

  int x, y;
  GetPosition(&x, &y);
  SetSize(x,y, width_, height_+80);
}


void ColorGuiFrame::OnReset(wxCommandEvent &event)
{
    int iIdxDisplay = getVisionColor();
    if (iIdxDisplay==-1) return;

    vision_->setThreshold(iIdxDisplay, 0, 0, 0, 0, 0, 0);
    rgbText_->SetValue(wxString::FromAscii("(no color selected)"));
    yuvText_->SetValue(wxString::FromAscii("(no bounds defined)"));
}

void ColorGuiFrame::OnSave(wxCommandEvent &event)
{
    if (sConfigFile.empty()) {
        wxMessageBox(wxT("Sorry, no colors filename to save was provided."), wxT("Save"), wxOK);
        return;
    }
    std::stringstream ssFormat;
    if (!vision_->saveOptions(sConfigFile.c_str())) {
        ssFormat <<  "Sorry, failed to save color file '" << sConfigFile << "'";
        wxMessageBox(wxString::FromAscii(ssFormat.str().c_str()), wxT("Save"), wxOK);
        return;
    }
    ssFormat << "Saved color file '" << sConfigFile << "'";
    ROS_INFO("%s", ssFormat.str().c_str());
    SetStatusText(wxString::FromAscii(ssFormat.str().c_str()));
    iStatusCounter = TIMER_INTERVAL_DISPLAY;
}

void ColorGuiFrame::OnColorEnter(wxCommandEvent &event)
{
    std::string sColor;
    int iIdxDisplay = getVisionColor(&sColor);
    if (iIdxDisplay==-1) {
        CMVision::color_info colorNew;
        memset(&colorNew.color, 0, sizeof(rgb));
        strncpy(colorNew.name, sColor.c_str(), CMV_MAX_NAME);
        colorNew.merge = 0.0;           //TODO: update with a slider?
        colorNew.expected_num = 10;     //TODO: update with a slider?
        colorNew.y_low = colorNew.y_high = 0;
        colorNew.u_low = colorNew.u_high = 0;
        colorNew.v_low = colorNew.v_high = 0;
        iIdxDisplay = vision_->addColorinfo(colorNew);
        
        if (iIdxDisplay!=-1) {
            std::stringstream ssFormat;
            ssFormat << "Added new color '" << sColor << "' with index " << iIdxDisplay;
            ROS_INFO("%s", ssFormat.str().c_str());
            SetStatusText(wxString::FromAscii(sStatus.c_str()));
            ssFormat.str(""); ssFormat.clear();
            ssFormat << iIdxDisplay;
            colorCombo_->Append(wxString::FromAscii(sColor.c_str()));
            colorIdx_->SetValue(wxString::FromAscii(ssFormat.str().c_str()));
        }
        else {
            SetStatusText( wxT("Sorry, no more space in color table, aborting."));
        }
        iStatusCounter = TIMER_INTERVAL_DISPLAY;
    }
    if (iIdxDisplay!=-1) {
        wxCommandEvent evFake;
        OnReset(evFake);
    }
}

int ColorGuiFrame::getVisionColor(std::string *pRecvStr)
{
    std::string sColor = (const char*)colorCombo_->GetValue().mb_str(wxConvUTF8);
    if (pRecvStr) (*pRecvStr) = sColor;
    int retIdx = vision_->getColorIndex(sColor.c_str());
    if (sColor.empty()) retIdx=0;
    return retIdx;
}

void ColorGuiFrame::OnColorChange(wxCommandEvent &event)
{
    std::string sColor;
    int iIdxDisplay = getVisionColor(&sColor);
    if (iIdxDisplay==-1) return;

    std::stringstream ssFormat;
    CMVision::color_info *pInfo = NULL;
    pInfo = vision_->getColorInfo(iIdxDisplay);

    if (!pInfo) {
        ssFormat << "Unknown color '" << sColor << "' selected or not stored in vision.";
        rgbText_->SetValue(wxT("(unknown)"));
        yuvText_->SetValue(wxT("(unknown)"));
        colorIdx_->SetValue(wxT("-1"));
        SetStatusText(wxString::FromAscii(ssFormat.str().c_str()));
        iStatusCounter = TIMER_INTERVAL_DISPLAY;
        return;
    }

    ssFormat << iIdxDisplay;
    colorIdx_->SetValue(wxString::FromAscii(ssFormat.str().c_str()));
    
    ssFormat.clear(); ssFormat.str("");
    ssFormat << (int)pInfo->color.red << " "  << (int)pInfo->color.green << " " << (int)pInfo->color.blue;
    rgbText_->SetValue(wxString::FromAscii(ssFormat.str().c_str()));
    
    y_low_last = pInfo->y_low; y_high_last = pInfo->y_high; u_low_last = pInfo->u_low;
    u_high_last = pInfo->u_high; v_low_last = pInfo->v_low; v_high_last = pInfo->v_high;

    ssFormat.clear(); ssFormat.str("");
    ssFormat << pInfo->y_low << ":" << pInfo->y_high << ", "
            << pInfo->u_low << ":" << pInfo->u_high << ", "
            << pInfo->v_low << ":" << pInfo->v_high;
    yuvText_->SetValue(wxString::FromAscii(ssFormat.str().c_str()));
}

void ColorGuiFrame::OnUndo(wxCommandEvent &event)
{
    int y_low, y_high, u_low, u_high, v_low, v_high;
    if (y_low_last<0 && u_high_last<0) {
        wxMessageBox(wxT("Sorry, only one undo action is possible."), wxT("Undo"), wxOK);
        return;
    }

    int iIdxDisplay = getVisionColor();
    vision_->setThreshold(iIdxDisplay, y_low_last, y_high_last, u_low_last,
                          u_high_last, v_low_last, v_high_last);
    
    std::ostringstream stream;
    stream << y_low_last << ":" << y_high_last << ", "
            << u_low_last << ":" << u_high_last << ", "
            << v_low_last << ":" << v_high_last;
    
    rgbText_->SetValue(wxT("(invalidated by last undo)"));
    yuvText_->SetValue(wxString::FromAscii(stream.str().c_str()));
    y_low_last = y_high_last = u_low_last = u_high_last = v_low_last = v_high_last = -1;
}

void ColorGuiFrame::OnClick(wxMouseEvent &event)
{
  int r, g, b, y, u, v;

  int px = (event.m_x/scale_) + ((scale_pos_x_*scale_) - scale_pos_x_)/scale_;
  int py = (event.m_y/scale_) + ((scale_pos_y_*scale_) - scale_pos_y_)/scale_;

  r = rgb_image_[py * (width_ * 3) + px * 3 + 0];
  g = rgb_image_[py * (width_ * 3) + px * 3 + 1];
  b = rgb_image_[py * (width_ * 3) + px * 3 + 2];

  std::ostringstream stream1;
  stream1 <<  r << " "  << " " << g << " " << b;
  rgbText_->SetValue(wxString::FromAscii(stream1.str().c_str()));
    
  RGB2YUV(r, g, b, y, u, v);

  int y_low, y_high, u_low, u_high, v_low, v_high;
    
  int iIdxDisplay = getVisionColor();
  vision_->getThreshold(iIdxDisplay, y_low, y_high, u_low, u_high, v_low, v_high);
  rgb rgbPrior = vision_->getColorVisual(iIdxDisplay);
  if (rgbPrior.red==0 && rgbPrior.blue==0 && rgbPrior.green==0) {
      CMVision::color_info infoPrior;
      vision_->getColorInfo(iIdxDisplay, infoPrior);
      infoPrior.color.red = r;
      infoPrior.color.blue = b;
      infoPrior.color.green = g;
      vision_->setColorInfo(iIdxDisplay, infoPrior);
      ROS_INFO("Updating primary color for '%s' -> %d,%d,%d", infoPrior.name, r, g, b);
  }
    
  y_low_last = y_low; y_high_last = y_high; u_low_last = u_low;
  u_high_last = u_high; v_low_last = v_low; v_high_last = v_high;
   
  if (y_low == 0 && y_high == 0)
  {
    y_low = y;
    y_high = y;
  }
  if (u_low == 0 && u_high == 0)
  {
    u_low = u;
    u_high = u;
  }
  if (v_low == 0 && v_high == 0)
  {
    v_low = v;
    v_high = v;
  }

  y_low = std::min(y, y_low);
  y_high = std::max(y, y_high);

  u_low = std::min(u, u_low);
  u_high = std::max(u, u_high);

  v_low = std::min(v, v_low);
  v_high = std::max(v, v_high);

  //save last value for undo function
  vision_->setThreshold(iIdxDisplay, y_low, y_high, u_low, u_high, v_low, v_high);

  std::ostringstream stream;
  stream << y_low << ":" << y_high << ", "
         << u_low << ":" << u_high << ", "
         << v_low << ":" << v_high;

  yuvText_->SetValue(wxString::FromAscii(stream.str().c_str()));
}

void ColorGuiFrame::OnMouseWheel(wxMouseEvent &event)
{
  if (event.GetWheelRotation() < 0)
    scale_ *= 0.9;
  else
    scale_ *= 1.1;

  scale_pos_x_ = event.m_x;
  scale_pos_y_ = event.m_y;
}
