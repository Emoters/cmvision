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

#include <ros/ros.h>
#include <opencv/cv.h>
#include <sensor_msgs/Image.h>
#include <wx/wx.h>

#include "cmvision.h"

class ColorGuiFrame : public wxFrame
{
  enum {ID_Reset=1,ID_Undo,ID_Save};

  /// \brief Constructor
  public: ColorGuiFrame(const char *szConfigFile=NULL);

  /// \brief timer called from app (don't block here)
  public: void OnTimer(void);

  /// \brief Quit callback
  public: void OnQuit(wxCommandEvent &event);

  /// \brief Reset callback
  public: void OnReset(wxCommandEvent &event);
    
  /// \brief Save callback
  public: void OnSave(wxCommandEvent &event);
    
  /// \brief Undo callback
  public: void OnUndo(wxCommandEvent &event);
    
  /// \brief On image click callback
  public: void OnClick(wxMouseEvent &event);
    
  /// \brief Callback for zooming
  public: void OnMouseWheel(wxMouseEvent &event);
  
  /// \brief Callback for color combo change
  public: void OnColorChange(wxCommandEvent &event);
    
  /// \brief Callback for enter in combo
  public: void OnColorEnter(wxCommandEvent &event);
    
  /// \brief Draw an image frame
  public: void DrawImage(const sensor_msgs::ImageConstPtr& msg);
    
  /// \brief fetch color from combo
  protected: int getVisionColor(std::string *pRecvStr=NULL);
    
  private: int width_, height_;
  private: wxTextCtrl *yuvText_;
  private: wxTextCtrl *rgbText_;
  private: wxTextCtrl *colorIdx_;
  private: wxComboBox *colorCombo_;
  private: wxPanel *image_panel_;

  private: unsigned char *rgb_image_;
  private: unsigned char *uyvy_image_;
  private: std::string sConfigFile;

  private: int scale_pos_x_, scale_pos_y_;
  private: float scale_;

  private: CMVision *vision_;
  private: int y_low_last, y_high_last, u_low_last, u_high_last, v_low_last, v_high_last;
  private: std::string sStatus;
  private: int iStatusCounter;
};


class ColorGuiApp : public wxApp
{
  /// \brief On init of the application
  public: bool OnInit();

  /// \brief On update, used for ros::spin
  public: void OnUpdate( wxTimerEvent &event );

  /// \brief The image callback
  private: void imageCB(const sensor_msgs::ImageConstPtr& msg);

  private: ros::Subscriber image_subscriber_;
  private: wxTimer *update_timer_;
  private: ColorGuiFrame *frame_;
};

DECLARE_APP(ColorGuiApp)
IMPLEMENT_APP(ColorGuiApp)
