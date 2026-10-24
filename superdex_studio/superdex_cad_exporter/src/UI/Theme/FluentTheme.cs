/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 */

using System.Windows;
using System.Windows.Markup;

namespace CADRobotExporter.UI.Theme
{
    /// <summary>
    /// A light Fluent (Windows 11) theme for the exporter's WPF windows.
    /// <para>
    /// Authored as XAML parsed at runtime rather than as C# object graphs: rounded corners require
    /// replacing each control's <c>ControlTemplate</c>, and templates expressed through
    /// <c>FrameworkElementFactory</c> are effectively unreadable. Runtime parsing also keeps the
    /// project's existing decision to have no XAML build items, so no markup compilation is needed.
    /// </para>
    /// </summary>
    public static class FluentTheme
    {
        /// <summary>
        /// Merges the theme into <paramref name="target"/>'s own resources.
        /// </summary>
        /// <remarks>
        /// A fresh dictionary is parsed per call rather than caching one shared instance. The
        /// progress window runs on its own STA thread and dispatcher, and the brushes and templates
        /// in here are unfrozen DispatcherObjects, so sharing one instance across both dispatchers
        /// would throw on first use from the wrong thread.
        /// </remarks>
        public static void Apply(FrameworkElement target)
        {
            if (target == null)
            {
                return;
            }

            target.Resources.MergedDictionaries.Add((ResourceDictionary)XamlReader.Parse(Xaml));
        }

        /// <summary>Style key for the primary (accent-filled) button.</summary>
        public const string AccentButtonKey = "AccentButton";

        // Single quotes are used for XAML attribute values throughout so the C# verbatim string
        // does not need doubled quotes on every line.
        private const string Xaml = @"
<ResourceDictionary xmlns='http://schemas.microsoft.com/winfx/2006/xaml/presentation'
                    xmlns:x='http://schemas.microsoft.com/winfx/2006/xaml'
                    xmlns:sys='clr-namespace:System;assembly=mscorlib'>

  <!-- Windows 11 light palette. -->
  <Color x:Key='AccentColor'>#0067C0</Color>
  <SolidColorBrush x:Key='Accent' Color='#0067C0'/>
  <SolidColorBrush x:Key='AccentHover' Color='#1975C5'/>
  <SolidColorBrush x:Key='AccentPressed' Color='#3183CE'/>
  <!-- Tinted selection fill for list and tree rows; the solid accent is too heavy at row size. -->
  <SolidColorBrush x:Key='AccentSubtle' Color='#E4EEF7'/>

  <SolidColorBrush x:Key='LayerFill' Color='#F3F3F3'/>
  <SolidColorBrush x:Key='CardFill' Color='#FFFFFF'/>
  <SolidColorBrush x:Key='ControlFill' Color='#FDFDFD'/>
  <SolidColorBrush x:Key='ControlFillHover' Color='#F5F5F5'/>
  <SolidColorBrush x:Key='ControlFillPressed' Color='#F0F0F0'/>
  <SolidColorBrush x:Key='ControlFillDisabled' Color='#F5F5F5'/>
  <SolidColorBrush x:Key='SubtleHover' Color='#EAEAEA'/>

  <SolidColorBrush x:Key='Stroke' Color='#E5E5E5'/>
  <SolidColorBrush x:Key='StrokeStrong' Color='#CFCFCF'/>
  <SolidColorBrush x:Key='StrokeAccentBase' Color='#868686'/>

  <SolidColorBrush x:Key='TextPrimary' Color='#1B1B1B'/>
  <SolidColorBrush x:Key='TextSecondary' Color='#5D5D5D'/>
  <SolidColorBrush x:Key='TextDisabled' Color='#A0A0A0'/>
  <SolidColorBrush x:Key='TextOnAccent' Color='#FFFFFF'/>

  <!-- Segoe UI Variable ships with Windows 11; Segoe UI is the Windows 10 fallback. -->
  <FontFamily x:Key='UiFont'>Segoe UI Variable Text, Segoe UI</FontFamily>
  <sys:Double x:Key='UiFontSize'>13</sys:Double>
  <CornerRadius x:Key='ControlCorner'>4</CornerRadius>
  <CornerRadius x:Key='CardCorner'>7</CornerRadius>

  <!-- ============================ Text ============================ -->

  <Style TargetType='TextBlock'>
    <Setter Property='FontFamily' Value='{StaticResource UiFont}'/>
    <Setter Property='FontSize' Value='{StaticResource UiFontSize}'/>
    <Setter Property='Foreground' Value='{StaticResource TextPrimary}'/>
    <Setter Property='TextOptions.TextFormattingMode' Value='Ideal'/>
  </Style>

  <Style TargetType='Label'>
    <Setter Property='FontFamily' Value='{StaticResource UiFont}'/>
    <Setter Property='FontSize' Value='{StaticResource UiFontSize}'/>
    <Setter Property='Foreground' Value='{StaticResource TextPrimary}'/>
    <Setter Property='Padding' Value='0,2'/>
  </Style>

  <!-- ============================ Button ============================ -->

  <Style TargetType='Button'>
    <Setter Property='FontFamily' Value='{StaticResource UiFont}'/>
    <Setter Property='FontSize' Value='{StaticResource UiFontSize}'/>
    <Setter Property='Foreground' Value='{StaticResource TextPrimary}'/>
    <Setter Property='Background' Value='{StaticResource ControlFill}'/>
    <Setter Property='BorderBrush' Value='{StaticResource StrokeStrong}'/>
    <Setter Property='BorderThickness' Value='1'/>
    <Setter Property='Padding' Value='10,3'/>
    <Setter Property='MinHeight' Value='26'/>
    <Setter Property='HorizontalContentAlignment' Value='Center'/>
    <Setter Property='VerticalContentAlignment' Value='Center'/>
    <Setter Property='SnapsToDevicePixels' Value='True'/>
    <Setter Property='Template'>
      <Setter.Value>
        <ControlTemplate TargetType='Button'>
          <Border x:Name='Bd'
                  Background='{TemplateBinding Background}'
                  BorderBrush='{TemplateBinding BorderBrush}'
                  BorderThickness='{TemplateBinding BorderThickness}'
                  CornerRadius='{StaticResource ControlCorner}'
                  SnapsToDevicePixels='True'>
            <ContentPresenter x:Name='Cp'
                              Margin='{TemplateBinding Padding}'
                              HorizontalAlignment='{TemplateBinding HorizontalContentAlignment}'
                              VerticalAlignment='{TemplateBinding VerticalContentAlignment}'
                              RecognizesAccessKey='True'/>
          </Border>
          <ControlTemplate.Triggers>
            <Trigger Property='IsMouseOver' Value='True'>
              <Setter TargetName='Bd' Property='Background' Value='{StaticResource ControlFillHover}'/>
            </Trigger>
            <Trigger Property='IsPressed' Value='True'>
              <Setter TargetName='Bd' Property='Background' Value='{StaticResource ControlFillPressed}'/>
              <Setter TargetName='Cp' Property='Opacity' Value='0.6'/>
            </Trigger>
            <Trigger Property='IsKeyboardFocused' Value='True'>
              <Setter TargetName='Bd' Property='BorderBrush' Value='{StaticResource Accent}'/>
            </Trigger>
            <Trigger Property='IsEnabled' Value='False'>
              <Setter TargetName='Bd' Property='Background' Value='{StaticResource ControlFillDisabled}'/>
              <Setter TargetName='Bd' Property='BorderBrush' Value='{StaticResource Stroke}'/>
              <Setter Property='Foreground' Value='{StaticResource TextDisabled}'/>
            </Trigger>
          </ControlTemplate.Triggers>
        </ControlTemplate>
      </Setter.Value>
    </Setter>
  </Style>

  <Style x:Key='AccentButton' TargetType='Button' BasedOn='{StaticResource {x:Type Button}}'>
    <Setter Property='Background' Value='{StaticResource Accent}'/>
    <Setter Property='BorderBrush' Value='{StaticResource Accent}'/>
    <Setter Property='Foreground' Value='{StaticResource TextOnAccent}'/>
    <Setter Property='Template'>
      <Setter.Value>
        <ControlTemplate TargetType='Button'>
          <Border x:Name='Bd'
                  Background='{TemplateBinding Background}'
                  BorderBrush='{TemplateBinding BorderBrush}'
                  BorderThickness='{TemplateBinding BorderThickness}'
                  CornerRadius='{StaticResource ControlCorner}'
                  SnapsToDevicePixels='True'>
            <ContentPresenter x:Name='Cp'
                              Margin='{TemplateBinding Padding}'
                              HorizontalAlignment='Center'
                              VerticalAlignment='Center'
                              RecognizesAccessKey='True'/>
          </Border>
          <ControlTemplate.Triggers>
            <Trigger Property='IsMouseOver' Value='True'>
              <Setter TargetName='Bd' Property='Background' Value='{StaticResource AccentHover}'/>
              <Setter TargetName='Bd' Property='BorderBrush' Value='{StaticResource AccentHover}'/>
            </Trigger>
            <Trigger Property='IsPressed' Value='True'>
              <Setter TargetName='Bd' Property='Background' Value='{StaticResource AccentPressed}'/>
              <Setter TargetName='Bd' Property='BorderBrush' Value='{StaticResource AccentPressed}'/>
              <Setter TargetName='Cp' Property='Opacity' Value='0.7'/>
            </Trigger>
            <Trigger Property='IsEnabled' Value='False'>
              <Setter TargetName='Bd' Property='Background' Value='{StaticResource ControlFillDisabled}'/>
              <Setter TargetName='Bd' Property='BorderBrush' Value='{StaticResource Stroke}'/>
              <Setter Property='Foreground' Value='{StaticResource TextDisabled}'/>
            </Trigger>
          </ControlTemplate.Triggers>
        </ControlTemplate>
      </Setter.Value>
    </Setter>
  </Style>

  <!-- Compact flat button, used for the numeric spinner's up/down arrows. -->
  <Style TargetType='RepeatButton'>
    <Setter Property='FontFamily' Value='{StaticResource UiFont}'/>
    <Setter Property='FontSize' Value='9'/>
    <Setter Property='Foreground' Value='{StaticResource TextSecondary}'/>
    <Setter Property='Background' Value='Transparent'/>
    <Setter Property='BorderThickness' Value='0'/>
    <Setter Property='Padding' Value='0'/>
    <Setter Property='SnapsToDevicePixels' Value='True'/>
    <Setter Property='Template'>
      <Setter.Value>
        <ControlTemplate TargetType='RepeatButton'>
          <Border x:Name='Bd'
                  Background='{TemplateBinding Background}'
                  CornerRadius='3'
                  SnapsToDevicePixels='True'>
            <ContentPresenter HorizontalAlignment='Center' VerticalAlignment='Center'/>
          </Border>
          <ControlTemplate.Triggers>
            <Trigger Property='IsMouseOver' Value='True'>
              <Setter TargetName='Bd' Property='Background' Value='{StaticResource SubtleHover}'/>
            </Trigger>
            <Trigger Property='IsPressed' Value='True'>
              <Setter TargetName='Bd' Property='Background' Value='{StaticResource ControlFillPressed}'/>
            </Trigger>
            <Trigger Property='IsEnabled' Value='False'>
              <Setter Property='Foreground' Value='{StaticResource TextDisabled}'/>
            </Trigger>
          </ControlTemplate.Triggers>
        </ControlTemplate>
      </Setter.Value>
    </Setter>
  </Style>

  <!-- ============================ TextBox ============================ -->

  <Style TargetType='TextBox'>
    <Setter Property='FontFamily' Value='{StaticResource UiFont}'/>
    <Setter Property='FontSize' Value='{StaticResource UiFontSize}'/>
    <Setter Property='Foreground' Value='{StaticResource TextPrimary}'/>
    <Setter Property='Background' Value='{StaticResource CardFill}'/>
    <Setter Property='BorderBrush' Value='{StaticResource Stroke}'/>
    <Setter Property='BorderThickness' Value='1'/>
    <Setter Property='Padding' Value='6,1'/>
    <Setter Property='MinHeight' Value='22'/>
    <Setter Property='VerticalContentAlignment' Value='Center'/>
    <Setter Property='CaretBrush' Value='{StaticResource TextPrimary}'/>
    <Setter Property='SelectionBrush' Value='{StaticResource Accent}'/>
    <Setter Property='SnapsToDevicePixels' Value='True'/>
    <Setter Property='Template'>
      <Setter.Value>
        <ControlTemplate TargetType='TextBox'>
          <Grid>
            <Border x:Name='Bd'
                    Background='{TemplateBinding Background}'
                    BorderBrush='{TemplateBinding BorderBrush}'
                    BorderThickness='{TemplateBinding BorderThickness}'
                    CornerRadius='{StaticResource ControlCorner}'
                    SnapsToDevicePixels='True'/>
            <!-- Windows 11 draws a heavier line under text fields, and turns it accent on focus. -->
            <Border x:Name='Underline'
                    Height='1'
                    Margin='1,0'
                    VerticalAlignment='Bottom'
                    Background='{StaticResource StrokeAccentBase}'
                    CornerRadius='0,0,3,3'/>
            <ScrollViewer x:Name='PART_ContentHost'
                          Margin='{TemplateBinding Padding}'
                          VerticalAlignment='{TemplateBinding VerticalContentAlignment}'
                          BorderThickness='0'
                          Background='Transparent'/>
          </Grid>
          <ControlTemplate.Triggers>
            <Trigger Property='IsMouseOver' Value='True'>
              <Setter TargetName='Bd' Property='BorderBrush' Value='{StaticResource StrokeStrong}'/>
            </Trigger>
            <Trigger Property='IsKeyboardFocused' Value='True'>
              <Setter TargetName='Underline' Property='Height' Value='2'/>
              <Setter TargetName='Underline' Property='Background' Value='{StaticResource Accent}'/>
            </Trigger>
            <Trigger Property='IsEnabled' Value='False'>
              <Setter TargetName='Bd' Property='Background' Value='{StaticResource ControlFillDisabled}'/>
              <Setter TargetName='Underline' Property='Background' Value='{StaticResource Stroke}'/>
              <Setter Property='Foreground' Value='{StaticResource TextDisabled}'/>
            </Trigger>
            <Trigger Property='IsReadOnly' Value='True'>
              <Setter TargetName='Bd' Property='Background' Value='{StaticResource ControlFillDisabled}'/>
              <Setter TargetName='Underline' Property='Background' Value='{StaticResource Stroke}'/>
            </Trigger>
            <!-- Borderless text boxes are nested inside another chrome (the numeric spinner),
                 which draws the border itself; a second underline inside it looks like a defect. -->
            <Trigger Property='BorderThickness' Value='0'>
              <Setter TargetName='Underline' Property='Visibility' Value='Collapsed'/>
            </Trigger>
          </ControlTemplate.Triggers>
        </ControlTemplate>
      </Setter.Value>
    </Setter>
  </Style>

  <!-- ============================ ComboBox ============================ -->

  <Style x:Key='ComboToggle' TargetType='ToggleButton'>
    <Setter Property='Background' Value='Transparent'/>
    <Setter Property='BorderThickness' Value='0'/>
    <Setter Property='Focusable' Value='False'/>
    <Setter Property='IsTabStop' Value='False'/>
    <Setter Property='ClickMode' Value='Press'/>
    <Setter Property='Template'>
      <Setter.Value>
        <ControlTemplate TargetType='ToggleButton'>
          <Border Background='Transparent'/>
        </ControlTemplate>
      </Setter.Value>
    </Setter>
  </Style>

  <Style TargetType='ComboBox'>
    <Setter Property='FontFamily' Value='{StaticResource UiFont}'/>
    <Setter Property='FontSize' Value='{StaticResource UiFontSize}'/>
    <Setter Property='Foreground' Value='{StaticResource TextPrimary}'/>
    <Setter Property='Background' Value='{StaticResource ControlFill}'/>
    <Setter Property='BorderBrush' Value='{StaticResource StrokeStrong}'/>
    <Setter Property='BorderThickness' Value='1'/>
    <Setter Property='Padding' Value='6,1'/>
    <Setter Property='MinHeight' Value='22'/>
    <Setter Property='VerticalContentAlignment' Value='Center'/>
    <Setter Property='SnapsToDevicePixels' Value='True'/>
    <Setter Property='ScrollViewer.HorizontalScrollBarVisibility' Value='Disabled'/>
    <Setter Property='Template'>
      <Setter.Value>
        <ControlTemplate TargetType='ComboBox'>
          <Grid>
            <Border x:Name='Bd'
                    Background='{TemplateBinding Background}'
                    BorderBrush='{TemplateBinding BorderBrush}'
                    BorderThickness='{TemplateBinding BorderThickness}'
                    CornerRadius='{StaticResource ControlCorner}'
                    SnapsToDevicePixels='True'/>
            <ToggleButton Style='{StaticResource ComboToggle}'
                          IsChecked='{Binding IsDropDownOpen, Mode=TwoWay, RelativeSource={RelativeSource TemplatedParent}}'/>
            <Grid Margin='{TemplateBinding Padding}'>
              <Grid.ColumnDefinitions>
                <ColumnDefinition Width='*'/>
                <ColumnDefinition Width='Auto'/>
              </Grid.ColumnDefinitions>
              <ContentPresenter x:Name='ContentSite'
                                Grid.Column='0'
                                Content='{TemplateBinding SelectionBoxItem}'
                                ContentTemplate='{TemplateBinding SelectionBoxItemTemplate}'
                                VerticalAlignment='Center'
                                HorizontalAlignment='Left'
                                IsHitTestVisible='False'/>
              <Path x:Name='Chevron'
                    Grid.Column='1'
                    Margin='8,0,0,0'
                    VerticalAlignment='Center'
                    Data='M 0 0 L 4 4 L 8 0'
                    Stroke='{StaticResource TextSecondary}'
                    StrokeThickness='1.2'
                    IsHitTestVisible='False'/>
            </Grid>
            <Popup x:Name='PART_Popup'
                   AllowsTransparency='True'
                   Placement='Bottom'
                   VerticalOffset='2'
                   IsOpen='{TemplateBinding IsDropDownOpen}'
                   PopupAnimation='Fade'
                   Focusable='False'>
              <Border Background='{StaticResource CardFill}'
                      BorderBrush='{StaticResource Stroke}'
                      BorderThickness='1'
                      CornerRadius='{StaticResource ControlCorner}'
                      MinWidth='{TemplateBinding ActualWidth}'
                      MaxHeight='{TemplateBinding MaxDropDownHeight}'
                      Margin='0,0,6,6'>
                <Border.Effect>
                  <DropShadowEffect BlurRadius='12' ShadowDepth='2' Direction='270' Opacity='0.18' Color='#000000'/>
                </Border.Effect>
                <ScrollViewer Margin='3'>
                  <ItemsPresenter/>
                </ScrollViewer>
              </Border>
            </Popup>
          </Grid>
          <ControlTemplate.Triggers>
            <Trigger Property='IsMouseOver' Value='True'>
              <Setter TargetName='Bd' Property='Background' Value='{StaticResource ControlFillHover}'/>
            </Trigger>
            <Trigger Property='IsKeyboardFocused' Value='True'>
              <Setter TargetName='Bd' Property='BorderBrush' Value='{StaticResource Accent}'/>
            </Trigger>
            <Trigger Property='IsEnabled' Value='False'>
              <Setter TargetName='Bd' Property='Background' Value='{StaticResource ControlFillDisabled}'/>
              <Setter TargetName='Bd' Property='BorderBrush' Value='{StaticResource Stroke}'/>
              <Setter TargetName='Chevron' Property='Stroke' Value='{StaticResource TextDisabled}'/>
              <Setter Property='Foreground' Value='{StaticResource TextDisabled}'/>
            </Trigger>
          </ControlTemplate.Triggers>
        </ControlTemplate>
      </Setter.Value>
    </Setter>
  </Style>

  <Style TargetType='ComboBoxItem'>
    <Setter Property='FontFamily' Value='{StaticResource UiFont}'/>
    <Setter Property='FontSize' Value='{StaticResource UiFontSize}'/>
    <Setter Property='Foreground' Value='{StaticResource TextPrimary}'/>
    <Setter Property='Padding' Value='8,4'/>
    <Setter Property='SnapsToDevicePixels' Value='True'/>
    <Setter Property='Template'>
      <Setter.Value>
        <ControlTemplate TargetType='ComboBoxItem'>
          <Border x:Name='Bd'
                  Background='Transparent'
                  CornerRadius='{StaticResource ControlCorner}'
                  Padding='{TemplateBinding Padding}'
                  Margin='1'
                  SnapsToDevicePixels='True'>
            <ContentPresenter VerticalAlignment='Center'/>
          </Border>
          <ControlTemplate.Triggers>
            <Trigger Property='IsMouseOver' Value='True'>
              <Setter TargetName='Bd' Property='Background' Value='{StaticResource SubtleHover}'/>
            </Trigger>
            <Trigger Property='IsSelected' Value='True'>
              <Setter TargetName='Bd' Property='Background' Value='{StaticResource ControlFillPressed}'/>
            </Trigger>
          </ControlTemplate.Triggers>
        </ControlTemplate>
      </Setter.Value>
    </Setter>
  </Style>

  <!-- ============================ CheckBox ============================ -->

  <Style TargetType='CheckBox'>
    <Setter Property='FontFamily' Value='{StaticResource UiFont}'/>
    <Setter Property='FontSize' Value='{StaticResource UiFontSize}'/>
    <Setter Property='Foreground' Value='{StaticResource TextPrimary}'/>
    <Setter Property='VerticalContentAlignment' Value='Center'/>
    <Setter Property='SnapsToDevicePixels' Value='True'/>
    <Setter Property='Template'>
      <Setter.Value>
        <ControlTemplate TargetType='CheckBox'>
          <Grid Background='Transparent'>
            <Grid.ColumnDefinitions>
              <ColumnDefinition Width='Auto'/>
              <ColumnDefinition Width='*'/>
            </Grid.ColumnDefinitions>
            <Border x:Name='Box'
                    Grid.Column='0'
                    Width='16' Height='16'
                    Background='{StaticResource CardFill}'
                    BorderBrush='{StaticResource StrokeAccentBase}'
                    BorderThickness='1'
                    CornerRadius='{StaticResource ControlCorner}'
                    VerticalAlignment='Center'
                    SnapsToDevicePixels='True'>
              <Path x:Name='Tick'
                    Data='M 0 3 L 2.8 6 L 7.2 0.8'
                    Stroke='{StaticResource TextOnAccent}'
                    StrokeThickness='1.5'
                    StrokeEndLineCap='Round'
                    StrokeStartLineCap='Round'
                    HorizontalAlignment='Center'
                    VerticalAlignment='Center'
                    Visibility='Collapsed'/>
            </Border>
            <ContentPresenter Grid.Column='1'
                              Margin='6,0,0,0'
                              VerticalAlignment='Center'
                              RecognizesAccessKey='True'/>
          </Grid>
          <ControlTemplate.Triggers>
            <Trigger Property='IsMouseOver' Value='True'>
              <Setter TargetName='Box' Property='Background' Value='{StaticResource ControlFillHover}'/>
            </Trigger>
            <Trigger Property='IsChecked' Value='True'>
              <Setter TargetName='Box' Property='Background' Value='{StaticResource Accent}'/>
              <Setter TargetName='Box' Property='BorderBrush' Value='{StaticResource Accent}'/>
              <Setter TargetName='Tick' Property='Visibility' Value='Visible'/>
            </Trigger>
            <MultiTrigger>
              <MultiTrigger.Conditions>
                <Condition Property='IsChecked' Value='True'/>
                <Condition Property='IsMouseOver' Value='True'/>
              </MultiTrigger.Conditions>
              <Setter TargetName='Box' Property='Background' Value='{StaticResource AccentHover}'/>
              <Setter TargetName='Box' Property='BorderBrush' Value='{StaticResource AccentHover}'/>
            </MultiTrigger>
            <Trigger Property='IsKeyboardFocused' Value='True'>
              <Setter TargetName='Box' Property='BorderBrush' Value='{StaticResource Accent}'/>
            </Trigger>
            <Trigger Property='IsEnabled' Value='False'>
              <Setter TargetName='Box' Property='Background' Value='{StaticResource ControlFillDisabled}'/>
              <Setter TargetName='Box' Property='BorderBrush' Value='{StaticResource Stroke}'/>
              <Setter TargetName='Tick' Property='Stroke' Value='{StaticResource TextDisabled}'/>
              <Setter Property='Foreground' Value='{StaticResource TextDisabled}'/>
            </Trigger>
          </ControlTemplate.Triggers>
        </ControlTemplate>
      </Setter.Value>
    </Setter>
  </Style>

  <!-- ============================ GroupBox ============================ -->

  <!-- Windows 11 has no group box; the closest equivalent is a card with a heading. -->
  <Style TargetType='GroupBox'>
    <Setter Property='FontFamily' Value='{StaticResource UiFont}'/>
    <Setter Property='FontSize' Value='{StaticResource UiFontSize}'/>
    <Setter Property='Foreground' Value='{StaticResource TextPrimary}'/>
    <Setter Property='Background' Value='{StaticResource CardFill}'/>
    <Setter Property='BorderBrush' Value='{StaticResource Stroke}'/>
    <Setter Property='BorderThickness' Value='1'/>
    <Setter Property='Padding' Value='8'/>
    <Setter Property='SnapsToDevicePixels' Value='True'/>
    <Setter Property='Template'>
      <Setter.Value>
        <ControlTemplate TargetType='GroupBox'>
          <Border Background='{TemplateBinding Background}'
                  BorderBrush='{TemplateBinding BorderBrush}'
                  BorderThickness='{TemplateBinding BorderThickness}'
                  CornerRadius='{StaticResource CardCorner}'
                  SnapsToDevicePixels='True'>
            <Grid>
              <Grid.RowDefinitions>
                <RowDefinition Height='Auto'/>
                <RowDefinition Height='*'/>
              </Grid.RowDefinitions>
              <ContentPresenter Grid.Row='0'
                                Margin='8,5,8,0'
                                ContentSource='Header'
                                TextBlock.FontWeight='Bold'
                                TextBlock.Foreground='{StaticResource TextPrimary}'/>
              <ContentPresenter Grid.Row='1' Margin='{TemplateBinding Padding}'/>
            </Grid>
          </Border>
        </ControlTemplate>
      </Setter.Value>
    </Setter>
  </Style>

  <!-- ============================ Tabs ============================ -->

  <Style TargetType='TabControl'>
    <Setter Property='FontFamily' Value='{StaticResource UiFont}'/>
    <Setter Property='FontSize' Value='{StaticResource UiFontSize}'/>
    <Setter Property='Background' Value='Transparent'/>
    <Setter Property='BorderThickness' Value='0'/>
    <Setter Property='Padding' Value='0'/>
    <Setter Property='Template'>
      <Setter.Value>
        <ControlTemplate TargetType='TabControl'>
          <Grid>
            <Grid.RowDefinitions>
              <RowDefinition Height='Auto'/>
              <RowDefinition Height='*'/>
            </Grid.RowDefinitions>
            <!-- Inset by the content card's corner radius so the first tab starts where the
                 card's straight edge does, not over the curve. -->
            <TabPanel Grid.Row='0' Panel.ZIndex='1' Margin='8,0,0,-1' IsItemsHost='True'/>
            <Border Grid.Row='1'
                    Background='{StaticResource CardFill}'
                    BorderBrush='{StaticResource Stroke}'
                    BorderThickness='1'
                    CornerRadius='{StaticResource CardCorner}'>
              <ContentPresenter ContentSource='SelectedContent'/>
            </Border>
          </Grid>
        </ControlTemplate>
      </Setter.Value>
    </Setter>
  </Style>

  <Style TargetType='TabItem'>
    <Setter Property='FontFamily' Value='{StaticResource UiFont}'/>
    <Setter Property='FontSize' Value='{StaticResource UiFontSize}'/>
    <Setter Property='Foreground' Value='{StaticResource TextSecondary}'/>
    <Setter Property='Padding' Value='14,5'/>
    <Setter Property='SnapsToDevicePixels' Value='True'/>
    <Setter Property='Template'>
      <Setter.Value>
        <ControlTemplate TargetType='TabItem'>
          <!-- The inter-tab gap lives on the template root, not on the TabItem itself: TabPanel
               lays items out flush and clips away a child Margin, which chops the right corner
               off mid-curve. -->
          <Border x:Name='Bd'
                  Margin='0,0,3,0'
                  Background='Transparent'
                  CornerRadius='7,7,0,0'
                  SnapsToDevicePixels='True'>
            <ContentPresenter x:Name='Cp'
                              Margin='{TemplateBinding Padding}'
                              ContentSource='Header'
                              HorizontalAlignment='Center'
                              VerticalAlignment='Center'
                              RecognizesAccessKey='True'/>
          </Border>
          <ControlTemplate.Triggers>
            <Trigger Property='IsMouseOver' Value='True'>
              <Setter TargetName='Bd' Property='Background' Value='{StaticResource SubtleHover}'/>
            </Trigger>
            <Trigger Property='IsSelected' Value='True'>
              <Setter TargetName='Bd' Property='Background' Value='{StaticResource CardFill}'/>
              <Setter Property='Foreground' Value='{StaticResource TextPrimary}'/>
              <Setter Property='FontWeight' Value='SemiBold'/>
            </Trigger>
            <Trigger Property='IsEnabled' Value='False'>
              <Setter Property='Foreground' Value='{StaticResource TextDisabled}'/>
            </Trigger>
          </ControlTemplate.Triggers>
        </ControlTemplate>
      </Setter.Value>
    </Setter>
  </Style>

  <!-- ============================ Slider ============================ -->

  <Style x:Key='SliderThumb' TargetType='Thumb'>
    <Setter Property='Focusable' Value='False'/>
    <Setter Property='OverridesDefaultStyle' Value='True'/>
    <Setter Property='Template'>
      <Setter.Value>
        <ControlTemplate TargetType='Thumb'>
          <Grid Width='20' Height='20' Background='Transparent'>
            <Ellipse Fill='{StaticResource CardFill}'
                     Stroke='{StaticResource StrokeStrong}'
                     StrokeThickness='1'/>
            <Ellipse x:Name='Core' Width='10' Height='10' Fill='{StaticResource Accent}'/>
          </Grid>
          <ControlTemplate.Triggers>
            <Trigger Property='IsMouseOver' Value='True'>
              <Setter TargetName='Core' Property='Width' Value='12'/>
              <Setter TargetName='Core' Property='Height' Value='12'/>
            </Trigger>
            <Trigger Property='IsDragging' Value='True'>
              <Setter TargetName='Core' Property='Width' Value='8'/>
              <Setter TargetName='Core' Property='Height' Value='8'/>
            </Trigger>
          </ControlTemplate.Triggers>
        </ControlTemplate>
      </Setter.Value>
    </Setter>
  </Style>

  <Style x:Key='SliderTrackButton' TargetType='RepeatButton'>
    <Setter Property='Focusable' Value='False'/>
    <Setter Property='IsTabStop' Value='False'/>
    <Setter Property='OverridesDefaultStyle' Value='True'/>
    <Setter Property='Template'>
      <Setter.Value>
        <ControlTemplate TargetType='RepeatButton'>
          <Border Background='Transparent'/>
        </ControlTemplate>
      </Setter.Value>
    </Setter>
  </Style>

  <Style TargetType='Slider'>
    <Setter Property='MinHeight' Value='20'/>
    <Setter Property='SnapsToDevicePixels' Value='True'/>
    <Setter Property='Template'>
      <Setter.Value>
        <ControlTemplate TargetType='Slider'>
          <Grid Background='Transparent' VerticalAlignment='Center'>
            <Border Height='4'
                    VerticalAlignment='Center'
                    Background='{StaticResource StrokeStrong}'
                    CornerRadius='2'/>
            <Track x:Name='PART_Track'>
              <Track.DecreaseRepeatButton>
                <RepeatButton Style='{StaticResource SliderTrackButton}' Command='Slider.DecreaseLarge'/>
              </Track.DecreaseRepeatButton>
              <Track.Thumb>
                <Thumb Style='{StaticResource SliderThumb}'/>
              </Track.Thumb>
              <Track.IncreaseRepeatButton>
                <RepeatButton Style='{StaticResource SliderTrackButton}' Command='Slider.IncreaseLarge'/>
              </Track.IncreaseRepeatButton>
            </Track>
          </Grid>
          <ControlTemplate.Triggers>
            <Trigger Property='IsEnabled' Value='False'>
              <Setter Property='Opacity' Value='0.5'/>
            </Trigger>
          </ControlTemplate.Triggers>
        </ControlTemplate>
      </Setter.Value>
    </Setter>
  </Style>

  <!-- ============================ ScrollBar ============================ -->

  <Style x:Key='ScrollThumb' TargetType='Thumb'>
    <Setter Property='OverridesDefaultStyle' Value='True'/>
    <Setter Property='IsTabStop' Value='False'/>
    <Setter Property='Template'>
      <Setter.Value>
        <ControlTemplate TargetType='Thumb'>
          <Border x:Name='Bd'
                  Background='#B0B0B0'
                  CornerRadius='3'
                  Margin='3'/>
          <ControlTemplate.Triggers>
            <Trigger Property='IsMouseOver' Value='True'>
              <Setter TargetName='Bd' Property='Background' Value='#8A8A8A'/>
            </Trigger>
          </ControlTemplate.Triggers>
        </ControlTemplate>
      </Setter.Value>
    </Setter>
  </Style>

  <Style TargetType='ScrollBar'>
    <Setter Property='Background' Value='Transparent'/>
    <Setter Property='Width' Value='12'/>
    <Setter Property='Template'>
      <Setter.Value>
        <ControlTemplate TargetType='ScrollBar'>
          <Grid Background='{TemplateBinding Background}'>
            <Track x:Name='PART_Track' IsDirectionReversed='True'>
              <Track.DecreaseRepeatButton>
                <RepeatButton Style='{StaticResource SliderTrackButton}' Command='ScrollBar.PageUpCommand'/>
              </Track.DecreaseRepeatButton>
              <Track.Thumb>
                <Thumb Style='{StaticResource ScrollThumb}'/>
              </Track.Thumb>
              <Track.IncreaseRepeatButton>
                <RepeatButton Style='{StaticResource SliderTrackButton}' Command='ScrollBar.PageDownCommand'/>
              </Track.IncreaseRepeatButton>
            </Track>
          </Grid>
        </ControlTemplate>
      </Setter.Value>
    </Setter>
    <Style.Triggers>
      <Trigger Property='Orientation' Value='Horizontal'>
        <Setter Property='Width' Value='Auto'/>
        <Setter Property='Height' Value='12'/>
        <Setter Property='Template'>
          <Setter.Value>
            <ControlTemplate TargetType='ScrollBar'>
              <Grid Background='{TemplateBinding Background}'>
                <Track x:Name='PART_Track'>
                  <Track.DecreaseRepeatButton>
                    <RepeatButton Style='{StaticResource SliderTrackButton}' Command='ScrollBar.PageLeftCommand'/>
                  </Track.DecreaseRepeatButton>
                  <Track.Thumb>
                    <Thumb Style='{StaticResource ScrollThumb}'/>
                  </Track.Thumb>
                  <Track.IncreaseRepeatButton>
                    <RepeatButton Style='{StaticResource SliderTrackButton}' Command='ScrollBar.PageRightCommand'/>
                  </Track.IncreaseRepeatButton>
                </Track>
              </Grid>
            </ControlTemplate>
          </Setter.Value>
        </Setter>
      </Trigger>
    </Style.Triggers>
  </Style>

  <!-- ============================ DataGrid ============================ -->

  <Style TargetType='DataGrid'>
    <Setter Property='FontFamily' Value='{StaticResource UiFont}'/>
    <Setter Property='FontSize' Value='{StaticResource UiFontSize}'/>
    <Setter Property='Foreground' Value='{StaticResource TextPrimary}'/>
    <Setter Property='Background' Value='{StaticResource CardFill}'/>
    <Setter Property='BorderBrush' Value='{StaticResource Stroke}'/>
    <Setter Property='BorderThickness' Value='1'/>
    <Setter Property='RowBackground' Value='Transparent'/>
    <Setter Property='AlternatingRowBackground' Value='#FAFAFA'/>
    <Setter Property='GridLinesVisibility' Value='Horizontal'/>
    <Setter Property='HorizontalGridLinesBrush' Value='{StaticResource Stroke}'/>
    <Setter Property='RowHeaderWidth' Value='0'/>
    <Setter Property='SnapsToDevicePixels' Value='True'/>
  </Style>

  <Style TargetType='DataGridColumnHeader'>
    <Setter Property='FontFamily' Value='{StaticResource UiFont}'/>
    <Setter Property='FontSize' Value='{StaticResource UiFontSize}'/>
    <Setter Property='FontWeight' Value='SemiBold'/>
    <Setter Property='Foreground' Value='{StaticResource TextSecondary}'/>
    <Setter Property='Background' Value='Transparent'/>
    <Setter Property='Padding' Value='8,5'/>
    <Setter Property='HorizontalContentAlignment' Value='Left'/>
    <Setter Property='Template'>
      <Setter.Value>
        <ControlTemplate TargetType='DataGridColumnHeader'>
          <Border Background='{TemplateBinding Background}'
                  BorderBrush='{StaticResource Stroke}'
                  BorderThickness='0,0,0,1'
                  Padding='{TemplateBinding Padding}'>
            <ContentPresenter VerticalAlignment='Center'/>
          </Border>
        </ControlTemplate>
      </Setter.Value>
    </Setter>
  </Style>

  <Style TargetType='DataGridCell'>
    <Setter Property='Padding' Value='8,3'/>
    <Setter Property='BorderThickness' Value='0'/>
    <Setter Property='Foreground' Value='{StaticResource TextPrimary}'/>
    <Setter Property='Template'>
      <Setter.Value>
        <ControlTemplate TargetType='DataGridCell'>
          <Border Background='{TemplateBinding Background}'
                  Padding='{TemplateBinding Padding}'
                  SnapsToDevicePixels='True'>
            <ContentPresenter VerticalAlignment='Center'/>
          </Border>
        </ControlTemplate>
      </Setter.Value>
    </Setter>
    <Style.Triggers>
      <Trigger Property='IsSelected' Value='True'>
        <Setter Property='Background' Value='#CCE4F7'/>
        <Setter Property='Foreground' Value='{StaticResource TextPrimary}'/>
      </Trigger>
    </Style.Triggers>
  </Style>

  <Style TargetType='DataGridRow'>
    <Setter Property='SnapsToDevicePixels' Value='True'/>
    <Style.Triggers>
      <Trigger Property='IsMouseOver' Value='True'>
        <Setter Property='Background' Value='{StaticResource SubtleHover}'/>
      </Trigger>
    </Style.Triggers>
  </Style>

  <!-- ============================ Misc ============================ -->

  <Style TargetType='ProgressBar'>
    <Setter Property='Height' Value='6'/>
    <Setter Property='Background' Value='{StaticResource Stroke}'/>
    <Setter Property='Foreground' Value='{StaticResource Accent}'/>
    <Setter Property='BorderThickness' Value='0'/>
    <Setter Property='Template'>
      <Setter.Value>
        <ControlTemplate TargetType='ProgressBar'>
          <Grid>
            <Border Background='{TemplateBinding Background}' CornerRadius='3'/>
            <Border x:Name='PART_Track'/>
            <Border x:Name='PART_Indicator'
                    HorizontalAlignment='Left'
                    Background='{TemplateBinding Foreground}'
                    CornerRadius='3'/>
          </Grid>
        </ControlTemplate>
      </Setter.Value>
    </Setter>
  </Style>

  <Style TargetType='ToolTip'>
    <Setter Property='FontFamily' Value='{StaticResource UiFont}'/>
    <Setter Property='FontSize' Value='{StaticResource UiFontSize}'/>
    <Setter Property='Foreground' Value='{StaticResource TextPrimary}'/>
    <Setter Property='Template'>
      <Setter.Value>
        <ControlTemplate TargetType='ToolTip'>
          <Border Background='{StaticResource CardFill}'
                  BorderBrush='{StaticResource Stroke}'
                  BorderThickness='1'
                  CornerRadius='{StaticResource ControlCorner}'
                  Padding='10,6'>
            <Border.Effect>
              <DropShadowEffect BlurRadius='10' ShadowDepth='2' Direction='270' Opacity='0.16' Color='#000000'/>
            </Border.Effect>
            <ContentPresenter/>
          </Border>
        </ControlTemplate>
      </Setter.Value>
    </Setter>
  </Style>

  <Style TargetType='Hyperlink'>
    <Setter Property='Foreground' Value='{StaticResource Accent}'/>
    <Setter Property='TextDecorations' Value='{x:Null}'/>
    <Style.Triggers>
      <Trigger Property='IsMouseOver' Value='True'>
        <Setter Property='TextDecorations' Value='Underline'/>
        <Setter Property='Foreground' Value='{StaticResource AccentHover}'/>
      </Trigger>
    </Style.Triggers>
  </Style>

  <Style TargetType='GridSplitter'>
    <Setter Property='Background' Value='Transparent'/>
  </Style>

  <Style TargetType='TreeView'>
    <Setter Property='Background' Value='{StaticResource CardFill}'/>
    <Setter Property='BorderBrush' Value='{StaticResource Stroke}'/>
    <Setter Property='BorderThickness' Value='1'/>
    <Setter Property='Padding' Value='4'/>
    <Setter Property='FontFamily' Value='{StaticResource UiFont}'/>
    <Setter Property='FontSize' Value='{StaticResource UiFontSize}'/>
    <Setter Property='Foreground' Value='{StaticResource TextPrimary}'/>
    <Setter Property='ScrollViewer.HorizontalScrollBarVisibility' Value='Auto'/>
    <Setter Property='ScrollViewer.VerticalScrollBarVisibility' Value='Auto'/>
    <Setter Property='Template'>
      <Setter.Value>
        <ControlTemplate TargetType='TreeView'>
          <Border Background='{TemplateBinding Background}'
                  BorderBrush='{TemplateBinding BorderBrush}'
                  BorderThickness='{TemplateBinding BorderThickness}'
                  CornerRadius='{StaticResource CardCorner}'>
            <ScrollViewer Focusable='False'
                          Padding='{TemplateBinding Padding}'
                          HorizontalScrollBarVisibility='Auto'
                          VerticalScrollBarVisibility='Auto'>
              <ItemsPresenter/>
            </ScrollViewer>
          </Border>
        </ControlTemplate>
      </Setter.Value>
    </Setter>
  </Style>

  <!-- The expander chevron is a plain glyph rather than the classic +/- box. -->
  <Style x:Key='TreeExpander' TargetType='ToggleButton'>
    <Setter Property='Focusable' Value='False'/>
    <Setter Property='Width' Value='16'/>
    <Setter Property='Height' Value='16'/>
    <Setter Property='Template'>
      <Setter.Value>
        <ControlTemplate TargetType='ToggleButton'>
          <Border Background='Transparent' Width='16' Height='16'>
            <Path x:Name='Chevron'
                  HorizontalAlignment='Center' VerticalAlignment='Center'
                  Data='M 0 0 L 4 4 L 0 8'
                  Stroke='{StaticResource TextSecondary}' StrokeThickness='1.4'
                  StrokeStartLineCap='Round' StrokeEndLineCap='Round'
                  RenderTransformOrigin='0.3,0.5'>
              <Path.RenderTransform>
                <RotateTransform Angle='0'/>
              </Path.RenderTransform>
            </Path>
          </Border>
          <ControlTemplate.Triggers>
            <Trigger Property='IsChecked' Value='True'>
              <Setter TargetName='Chevron' Property='RenderTransform'>
                <Setter.Value>
                  <RotateTransform Angle='90'/>
                </Setter.Value>
              </Setter>
            </Trigger>
            <Trigger Property='IsMouseOver' Value='True'>
              <Setter TargetName='Chevron' Property='Stroke' Value='{StaticResource TextPrimary}'/>
            </Trigger>
          </ControlTemplate.Triggers>
        </ControlTemplate>
      </Setter.Value>
    </Setter>
  </Style>

  <Style TargetType='TreeViewItem'>
    <Setter Property='IsSelected' Value='{Binding IsSelected, Mode=TwoWay}'/>
    <Setter Property='IsExpanded' Value='{Binding IsExpanded, Mode=TwoWay}'/>
    <Setter Property='Padding' Value='0'/>
    <Setter Property='Foreground' Value='{StaticResource TextPrimary}'/>
    <Setter Property='HorizontalContentAlignment' Value='Left'/>
    <Setter Property='VerticalContentAlignment' Value='Center'/>
    <Setter Property='Template'>
      <Setter.Value>
        <ControlTemplate TargetType='TreeViewItem'>
          <Grid>
            <Grid.RowDefinitions>
              <RowDefinition Height='Auto'/>
              <RowDefinition Height='Auto'/>
            </Grid.RowDefinitions>

            <!-- The row highlight spans both columns so it reads as one band, the way the
                 Windows 11 navigation pane does, rather than boxing just the label. The
                 expander and header are children of Bd, not siblings: IsMouseOver only counts
                 hits on an element and its descendants, so as siblings they would cover Bd and
                 suppress the hover trigger everywhere except the strips above and below them. -->
            <Border x:Name='Bd' Grid.Row='0'
                    Background='Transparent'
                    CornerRadius='{StaticResource ControlCorner}'
                    MinHeight='22'>
              <Grid>
                <Grid.ColumnDefinitions>
                  <ColumnDefinition Width='Auto' MinWidth='16'/>
                  <ColumnDefinition Width='*'/>
                </Grid.ColumnDefinitions>

                <ToggleButton x:Name='Expander' Grid.Column='0'
                              Style='{StaticResource TreeExpander}'
                              VerticalAlignment='Center'
                              ClickMode='Press'
                              IsChecked='{Binding IsExpanded, RelativeSource={RelativeSource TemplatedParent}}'/>

                <ContentPresenter x:Name='PART_Header' Grid.Column='1'
                                  ContentSource='Header'
                                  VerticalAlignment='Center'
                                  Margin='2,1,8,1'/>
              </Grid>
            </Border>

            <ItemsPresenter x:Name='ItemsHost' Grid.Row='1' Margin='16,0,0,0'/>
          </Grid>
          <ControlTemplate.Triggers>
            <Trigger Property='IsExpanded' Value='False'>
              <Setter TargetName='ItemsHost' Property='Visibility' Value='Collapsed'/>
            </Trigger>
            <Trigger Property='HasItems' Value='False'>
              <Setter TargetName='Expander' Property='Visibility' Value='Hidden'/>
            </Trigger>
            <Trigger Property='IsMouseOver' SourceName='Bd' Value='True'>
              <Setter TargetName='Bd' Property='Background' Value='{StaticResource ControlFillHover}'/>
            </Trigger>
            <Trigger Property='IsSelected' Value='True'>
              <Setter TargetName='Bd' Property='Background' Value='{StaticResource AccentSubtle}'/>
            </Trigger>
            <MultiTrigger>
              <MultiTrigger.Conditions>
                <Condition Property='IsSelected' Value='True'/>
                <Condition Property='IsSelectionActive' Value='False'/>
              </MultiTrigger.Conditions>
              <Setter TargetName='Bd' Property='Background' Value='{StaticResource ControlFillPressed}'/>
            </MultiTrigger>
            <Trigger Property='IsEnabled' Value='False'>
              <Setter Property='Foreground' Value='{StaticResource TextDisabled}'/>
            </Trigger>
          </ControlTemplate.Triggers>
        </ControlTemplate>
      </Setter.Value>
    </Setter>
  </Style>

</ResourceDictionary>
";
    }
}
