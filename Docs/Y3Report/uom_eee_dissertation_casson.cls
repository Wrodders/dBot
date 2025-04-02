%% Alex Casson - alex.casson@manchester.ac.uk
%
% Aim
% A LaTeX class file for submitting an MSc dissertation to the University of Manchester thesis, particularly for courses in Electrical and Electronic Engineering
%
% Versions
% 25.09.23 - v4 - updated font and font size
% 13.07.21 - v3 - new version based upon thesis template to make this more of a class than an example
% 03.08.20 - v2 - initial version released on Overleaf by Farshad
% 01.06.20 - v1 - Beta testing only - never publicly released
% 
% Notes
% - Assumes build with XeLaTeX or LuaLaTeX. Font setting is different for LaTeX or PDFLaTeX
% - On Linux systems requires ttf-mscorefonts-installer to be installed to have access to the correct fonts
% - Based upon 2023 submission guidelines (https://documents.manchester.ac.uk/DocuInfo.aspx?DocID=2863)
% - Assumes title page is only one page long!
% - Assumes not using subfigure. Tocloft has (easily fixed) issues if do. 
% - Note that for the references font size it is set here to be one larger than that actually wanted. Not clear why, but then gives the correct size! Have to do this in main .tex
%
% To do
% - Caption size is fixed at 10pt for all fonts rather than changing with each font
% -----------------------------------------------------------------------------


%% Class definition - based upon report class
\NeedsTeXFormat{LaTeX2e}
\ProvidesClass{uom_eee_dissertation_casson}[2023/09/25 - Alex Casson - University of Manchester MSc dissertation format]
\LoadClass[a4paper, oneside, 11pt]{article}


%% Load packages
%% Load packages
\RequirePackage[a-3u]{pdfx}
%\RequirePackage{hyperref}                % create internal hyperlinks, must be specified before cite package to work. Is loaded automatically by pdfx
  \hypersetup{hidelinks,colorlinks=false,breaklinks=true,bookmarksdepth=2}  % have no highlighting on links and break lines in the LOF and LOT
\RequirePackage[explicit]{titlesec}      % re-format section titles
\RequirePackage{geometry}                % set paper margins
\RequirePackage{setspace}                % set line spacing
\RequirePackage[document]{ragged2e}      % set to be left aligned
\RequirePackage{pgfcore}                 % for logo positioning in fixed location
\RequirePackage{fontspec}                % for font setting
\RequirePackage{tocloft} % allows precise control over the table of contents and deals with subfigures properly
  \renewcommand{\listfigurename}{List of figures} 
  \renewcommand{\listtablename}{List of tables} 
  \renewcommand{\cftsecleader}{\cftdotfill{\cftdotsep}} % add dots 
  \renewcommand{\cftsubsecleader}{\cftdotfill{\cftdotsep}}
  \setlength{\cftfignumwidth}{3em}  % modify number width in LoF
  \setlength{\cfttabnumwidth}{3em}  % modify number width in LoT
  \setlength{\cftfigindent}{0em}    % no indents in LoF
  \setlength{\cfttabindent}{0em}    % no indents in LoT
\RequirePackage{bookmark}                % for adding custom bookmarks
  \bookmarksetup{numbered}
\RequirePackage{enumitem}                % allow change of numbering style in enumeration
\RequirePackage{ifthen}                  % allow if then else statements
\RequirePackage{appendix}                % control over appendix
\RequirePackage{caption}                 % control figure captions

  
%% Handle options
%\DeclareOption{calibri}{\setmainfont{Calibri} \def\@@ptsize{12pt}}  
%\DeclareOption{course}{\renewcommand{\reportyear}{acse}} % option for setting course based on class switch. Not used at present
%\ProcessOptions\relax

%% Set font and size
\setmainfont{Arial} 
\@namedef{ds@11pt}{\def\@ptsize{11}}
\def\@@ptsize{11pt}


%% Accessibility options
%\RequirePackage{unicode-math} % unicode maths for accessibility
% Removed axessibility in 2022 version - 2022 version of texlive seems to have lots of bugs with this!
% Put the two lines low before decalre document class
%\RequirePackage{pdfmanagement-testphase}
%\DeclareDocumentMetadata{uncompress}
% Other lines go here
%\RequirePackage[tagpdf]{axessibility} % LaTeX readable equations for accessibility
%  \tagpdfsetup{tabsorder=structure,uncompress,activate-all,interwordspace=true}
%  \pdfextension catalog{/Lang (en-GB)} % note the language is also set on line 48, should match
% Can then use \alt command:
%\begin{figure}
%    \centering
%    \includegraphics[width=10cm]{apollo17_earth.jpg}
%    \alt{Image of the Earth from outer space}
%    \caption{Image of the Earth from the Apollo 17 mission}
%    \label{fig:earth}
%\end{figure}


%% Set lengths - sizes for A4 paper from http://www.brand.manchester.ac.uk/visual-identity/print/
\newlength{\logowidth}    \setlength{\logowidth}{40.006mm}
\newlength{\logoheight}   \setlength{\logoheight}{19.454mm}
\newlength{\logopad}      \setlength{\logopad}{14.279mm}
\newlength{\marginleft}   \setlength{\marginleft}{2cm}   % Minimum margins are 2cm
\newlength{\marginright}  \setlength{\marginright}{2cm} 
\newlength{\margintop}    \setlength{\margintop}{2cm}
\newlength{\marginbottom} \setlength{\marginbottom}{2cm}  


%% Set paper format
\geometry{a4paper, top=\margintop, left=\marginleft, right=\marginright, bottom=\marginbottom, nohead, nofoot, nomarginpar} 



%% Formatting options - mainly for font size
\setlength{\parskip}{2ex plus 0.5ex minus 0.2ex}
\setlength{\parindent}{0pt}
\linespread{1.5}
\pagestyle{plain} % simple page numbering
\AtBeginDocument{
  \fontsize{\@@ptsize}{\@@ptsize+1pt}\selectfont
}
\setcounter{tocdepth}{2}                % set toc depth. This is changed for the Appendix
\renewcommand{\topfraction}{0.85}       % depreciate use of full pages for images
\renewcommand{\textfraction}{0.1}
\renewcommand{\floatpagefraction}{0.75}
\setlength{\footskip}{26pt}

% Format captions
\DeclareCaptionFormat{myformat}{\fontsize{\ds@9pt}{\@@ptsize plus 1pt}\selectfont#1#2#3}
\captionsetup{justification=centering,labelformat=simple,labelsep=period,labelfont=bf,format=myformat}
\captionsetup[figure]{justification=centering,labelformat=simple,labelsep=period,labelfont=bf,name={Fig.},format=myformat}
 

%% Format titles
%\titleformat{\chapter}{\bfseries\Huge\raggedright}{\thechapter}{1ex}{#1} % default formatting for non-theses
%\titlespacing{\chapter}{0pt}{\@@ptsize}{\baselineskip}
\renewcommand{\contentsname}{\bfseries\Huge Contents}
\renewcommand{\listfigurename}{\bfseries\Huge List of figures}
\renewcommand{\listtablename}{\bfseries\Huge List of tables}
\titleformat{\section}{\bfseries\Large}{\thesection}{1ex}{#1}
\titlespacing{\section}{0pt}{\baselineskip}{0.2\baselineskip}
\titleformat{\subsection}{\bfseries\large}{\thesubsection}{1ex}{#1}


%% Title page
\makeatletter
\def\course#1{\gdef\@course{#1}}
\def\studentid#1{\gdef\@studentid{#1}}
\def\submitdate#1{\gdef\@submitdate{#1}}

\newcommand{\titletext}{Third Year Individual Project – Final Report} 

\newcommand{\submissiontext}{April 2025}

\newcommand{\authortext}{\textbf{\@author}\ \linebreak \@studentid}


\def\maketitle{
  \begin{titlepage}
    \hypertarget{TitlePage}{}
    \bookmark[dest=TitlePage,level=-1]{Front matter}
	\phantomsection \pdfbookmark[section]{Title page}{title}
  
    % Add logo. No requirement for this, but is nice. Note sizes are hand calculated. Won't be correct if change paper size
    \pgfdeclareimage[width=\logowidth]{uom_logo}{uom_logo}
    \pgftext[at=\pgfpoint{-\marginleft+\logopad}{\margintop+\parskip-\logopad}, top, left]{\pgfuseimage{uom_logo}}

    \begin{center}
	  \vspace*{\fill}
      {\Large \bf \@title}
      \vskip 2cm
      \titletext
      \vskip 2cm
      \submissiontext
	  \vskip 2cm
	  \authortext
	  \vskip 1cm
	  Supervisor:
    \vskip 0.5cm
    Dr. Joaquin Carrasco
    \vskip 2cm
      \vspace*{\fill}
    \end{center}

	
  \end{titlepage}
  
  \pagenumbering{arabic}
  \setcounter{page}{2} % ensure no number on title page, numbers on other pages match those in the PDF
}
\makeatother  %to avoid error messages generated by "\@". Makes Latex treat "@" like a letter



%% Abstract
\renewenvironment{abstract}{%
  \clearpage\phantomsection\section*{Abstract}
}
{\addcontentsline{toc}{section}{\abstractname}\endquotation\vfil\null\clearpage}



% Word count command
\makeatletter
\newcommand{\wordcount}[1]{\newcommand{\@wordcount}{#1}}
\makeatother
\makeatletter\newcommand{\thewordcount}[0]{\@wordcount}\makeatother



%% Lists and tables of contents
\newcommand{\uomtoc}{
  \phantomsection\addcontentsline{toc}{section}{Contents}
  \renewcommand*\contentsname{Contents}
  \tableofcontents
  \begin{flushright}
    \textbf{Word count}: \thewordcount
  \end{flushright}
  \clearpage
}


\newcommand{\uomlof}{
  \phantomsection\addcontentsline{toc}{section}{List of figures}
  \listoffigures
  \clearpage
  }  
\newcommand{\uomlot}{
  \phantomsection\addcontentsline{toc}{section}{List of tables}
  \listoftables
  \clearpage
  }    
\newenvironment{uomlop}{
  \phantomsection\addcontentsline{toc}{section}{List of publications}
  \section*{List of publications}
}
{\clearpage}
\newenvironment{uomterms}{
  \phantomsection\addcontentsline{toc}{section}{Terms and abbreviations}
  \section*{Terms and abbreviations}
}
{\clearpage}
\newenvironment{uomlay}{
  \phantomsection\addcontentsline{toc}{section}{Lay abstract}
  \section*{Lay abstract}
}
{\clearpage}

%% Declarations
\newcommand{\uomdeclarations}{
  \phantomsection\addcontentsline{toc}{section}{Declaration of originality}
  \section*{Declaration of originality}
  I hereby confirm that this dissertation is my own original work unless referenced clearly to the contrary, and that no portion of the work referred to in the dissertation has been submitted in support of an application for another degree or qualification of this or any other university or other institute of learning.
  \clearpage
  
  \phantomsection\addcontentsline{toc}{section}{Intellectual property statement}
  \section*{Intellectual property statement} 
  \begin{enumerate}[label=\roman*]
    \item The author of this thesis (including any appendices and/or schedules to this thesis) owns certain copyright or related rights in it (the ``Copyright'') and s/he has given The University of Manchester certain rights to use such Copyright, including for administrative purposes.
    \item Copies of this thesis, either in full or in extracts and whether in hard or electronic copy, may be made \emph{only} in accordance with the Copyright, Designs and Patents Act 1988 (as amended) and regulations issued under it or, where appropriate, in accordance with licensing agreements which the University has from time to time. This page must form part of any such copies made.
    \item The ownership of certain Copyright, patents, designs, trademarks and other intellectual property (the ``Intellectual Property'') and any reproductions of copyright works in the thesis, for example graphs and tables (``Reproductions''), which may be described in this thesis, may not be owned by the author and may be owned by third parties. Such Intellectual Property and Reproductions cannot and must not be made available for use without the prior written permission of the owner(s) of the relevant Intellectual Property and/or Reproductions.
	\item Further information on the conditions under which disclosure, publication and commercialisation of this thesis, the Copyright and any Intellectual Property and/or Reproductions described in it may take place is available in the University IP Policy (see \url{http://documents.manchester.ac.uk/DocuInfo.aspx?DocID=24420}), in any relevant Dissertation restriction declarations deposited in the University Library, and The University Library’s regulations (see \url{http://www.library.manchester.ac.uk/about/regulations/_files/Library-regulations.pdf}).
  \end{enumerate}
  \clearpage
}



%% Acknowledgements  
\newenvironment{uomacknowledgements}{
  \phantomsection\addcontentsline{toc}{section}{Acknowledgements}
  \section*{Acknowledgements}
  \bookmarksetup{startatroot}
}
{\clearpage}



%% Appendix
\newenvironment{uomappendix}{
  \clearpage
  \bookmarksetupnext{level=-1}
  \phantomsection\addcontentsline{toc}{section}{Appendices}
  \addtocontents{toc}{\protect\setcounter{tocdepth}{1}}
  \section*{Appendices} 
  %\clearpage
  %\renewcommand\sectionname{Appendix}
  \begin{appendices}
}
{\end{appendices}}