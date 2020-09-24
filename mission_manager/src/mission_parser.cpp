/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, QinetiQ, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of QinetiQ nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

// Original version: Christopher Scianna Christopher.Scianna@us.QinetiQ.com

#include "mission_parser.h"

//#include <libxml/parser.h>
//#include <libxml/xmlschemas.h>
#include <ros/console.h>
#include <stdarg.h>
#include <string.h>

#include "behavior.h"
#include "behavior_factory.h"
#include "mission.h"

//#define XML_WARN(node, fmt, ...) \
//    ROS_WARN("%s:%hu: "fmt, (const char *)xmlNodeGetBase(node->doc, node), node->line, ##__VA_ARGS__)

using namespace mission_manager;

static char *create_msg(const char *msg, va_list orig) {
  int n, size = 1024;
  char *p, *np;
  va_list ap;

  if ((p = (char *)malloc(size)) == NULL) return NULL;

  while (1) {
    va_copy(ap, orig);
    n = vsnprintf(p, size, msg, ap);
    va_end(ap);

    if ((n > -1) && (n < size)) return p;
    if (n > -1)
      size += n + 1;
    else
      size *= 2;
    if ((np = (char *)realloc(p, size)) == NULL) {
      free(p);
      return NULL;
    } else
      p = np;
  }
}

/*
static void print_warn(void __attribute__((unused)) *ctx, const char *msg, ...)
{
        va_list ap;
        char *buf = NULL;

        va_start(ap, msg);
        buf = create_msg(msg, ap);
        va_end(ap);

        if (buf != NULL) {
                ROS_WARN(buf);
                free(buf);
        }
}
*/

MissionParser::MissionParser() {}

MissionParser::MissionParser(ros::NodeHandle nh) : node_handle(nh) {}

MissionParser::~MissionParser() {}

/*
int MissionParser::validateMissionDoc(xmlDocPtr mission_doc, const std::string& schema_file)
{
        int ret = 0;
        xmlDocPtr schema_doc;
        xmlSchemaParserCtxtPtr parser_ctxt;
        xmlSchemaPtr schema;
        xmlSchemaValidCtxtPtr valid_ctxt;

        // Read the schema file
        schema_doc = xmlReadFile(schema_file.c_str(), NULL, XML_PARSE_NONET);
        if (schema_doc == NULL) {
                ROS_ERROR("%s(): Schema file could not be read.", __FUNCTION__);
                return -1;
        }

        // Set up all of the objects needed for schema validation
        parser_ctxt = xmlSchemaNewDocParserCtxt(schema_doc);
        if (parser_ctxt == NULL) {
                ROS_ERROR("%s(): Unable to create parser context for schema.", __FUNCTION__);
                ret = -1;
                goto free_sdoc;
        }

        schema = xmlSchemaParse(parser_ctxt);
        if (schema == NULL) {
                ROS_ERROR("%s(): The schema is not valid.", __FUNCTION__);
                ret = -1;
                goto free_parser;
        }

        valid_ctxt = xmlSchemaNewValidCtxt(schema);
        if (valid_ctxt == NULL) {
                ROS_ERROR("%s(): Unable to create validation context for schema.", __FUNCTION__);
                ret = -1;
                goto free_schema;
        }

        // Set up some handlers so we can see the parsing and validation warnings/errors
        xmlSchemaSetParserErrors(parser_ctxt, print_warn, print_warn, NULL);
        xmlSchemaSetValidErrors(valid_ctxt, print_warn, print_warn, NULL);

        // Validate the mission doc
        ret = xmlSchemaValidateDoc(valid_ctxt, mission_doc);

        // Clean up
        xmlSchemaFreeValidCtxt(valid_ctxt);
free_schema:
        xmlSchemaFree(schema);
free_parser:
        xmlSchemaFreeParserCtxt(parser_ctxt);
free_sdoc:
        xmlFreeDoc(schema_doc);

        return ret;
}
*/

bool MissionParser::parseMissionFile(Mission &mission, const std::string &mission_file) {
  int rc;

  TiXmlDocument doc(mission_file);

  try {
    if (!doc.LoadFile()) {
      ROS_ERROR("%s(): Could not read mission file.", __FUNCTION__);
      return false;
    }

    TiXmlElement *pMissionElement = doc.FirstChildElement("mission");
    if (pMissionElement == NULL) {
      ROS_WARN("Could not get mission element");
      return false;
    }

    // look for mission description
    TiXmlElement *pDesc = pMissionElement->FirstChildElement("description");
    if (pDesc != NULL) {
      ROS_INFO("Parsing Mission Description");
      const char *str = pDesc->GetText();
      if (str != NULL) {
        std::string descStr = str;
        mission.setMissionDescription(descStr);
      } else {
        std::string descStr = "NONE<>";
        mission.setMissionDescription(descStr);
        ROS_WARN("Mission Description not found. Setting to 'NONE<>'!");
      }
    } else {
      ROS_WARN("Could not get mission description");
    }

    TiXmlElement *pBehaviors = pMissionElement->FirstChildElement("behaviors");
    if (pBehaviors != NULL) {
      ROS_INFO("Parsing Behaviors");
      TiXmlElement *pABehavior = pBehaviors->FirstChildElement();
      // A behavior
      while (pABehavior != NULL) {
        std::string nameStr = pABehavior->Value();

        if (nameStr.compare("description") == 0) {
          pABehavior = pABehavior->NextSiblingElement();
          continue;
        }

        ROS_INFO("Creating %s behavior", nameStr.c_str());

        Behavior *new_behavior = m_factory.createBehavior((const char *)nameStr.c_str());
        if (new_behavior == NULL) {
          ROS_WARN("Behavior node does not have an object factory [%s].", nameStr.c_str());
          return false;
        }

        if (!new_behavior->getParams(node_handle)) {
          ROS_ERROR("Failed to get configuration parameters for behavior [%s].", nameStr.c_str());
          return false;
        }

        std::list<BehaviorXMLParam> paramXmlTags;  // = new_behavior->getBehaviorXmlParams();
        // std::list<BehaviorXMLParam>::iterator it = paramXmlTags.begin();

        TiXmlElement *pBehaviorParam = pABehavior->FirstChildElement();
        while (pBehaviorParam != NULL) {
          ROS_INFO("Element Name[%s], Value[%s]", pBehaviorParam->Value(),
                   pBehaviorParam->GetText());

          BehaviorXMLParam behParam;

          behParam.setXMLTag(pBehaviorParam->Value());  // this should return a string value like
                                                        // "waypoint", "attitude_servo", etc.
          const char *str = pBehaviorParam->GetText();
          if (str != NULL) {
            behParam.setXMLTagValue(str);
          }

          TiXmlAttribute *pAttrib = pBehaviorParam->FirstAttribute();

          while (pAttrib != NULL) {
            ROS_INFO("\t\t\t Attirbute Name[%s], Value[%s]", pAttrib->Name(), pAttrib->Value());
            behParam.addXMLTagAttribute(pAttrib->Name(), pAttrib->Value());
            pAttrib = pAttrib->Next();
          }

          paramXmlTags.push_back(behParam);

          pBehaviorParam = pBehaviorParam->NextSiblingElement();
        }

        new_behavior->setBehaviorXmlParams(paramXmlTags);
        if (new_behavior->parseMissionFileParams() == false) {
          ROS_INFO("Error parsing mission file parameters");
          return false;
        }

        // Route the parsing to the behavior library
        /*			if (!new_behavior->parseXml(behavior_node)) {
                                        ROS_ERROR("Failed to parse behavior node [%s].",
           nameStr.c_str()); ROS_WARN(behavior_node, "Failed to parse behavior node [%s].",
           nameStr.c_str()); return false;
                                }
        */
        // Add the node to the current mission
        mission.addBehavior(new_behavior);

        pABehavior = pABehavior->NextSiblingElement();
      }
    } else {
      ROS_WARN("Could not get behavoirs");
      return false;
    }

    TiXmlElement *pAbortBehaviors = pMissionElement->FirstChildElement("abort");
    if (pAbortBehaviors != NULL) {
      ROS_INFO("Parsing Abort Behaviors");
      TiXmlElement *pABehavior = pAbortBehaviors->FirstChildElement();
      while (pABehavior != NULL) {
        std::string nameStr = pABehavior->Value();

        if (nameStr.compare("description") == 0) {
          pABehavior = pABehavior->NextSiblingElement();
          continue;
        }

        ROS_INFO("Creating %s behavior", nameStr.c_str());

        Behavior *new_behavior = m_factory.createBehavior((const char *)nameStr.c_str());
        if (new_behavior == NULL) {
          ROS_WARN("Abort Behavior node does not have an object factory [%s].", nameStr.c_str());
          return false;
        }

        if (!new_behavior->getParams(node_handle)) {
          ROS_ERROR("Failed to get configuration parameters for abort behavior [%s].",
                    nameStr.c_str());
          return false;
        }

        std::list<BehaviorXMLParam> paramXmlTags;

        TiXmlElement *pBehaviorParam = pABehavior->FirstChildElement();
        while (pBehaviorParam != NULL) {
          ROS_INFO("Element Name[%s], Value[%s]", pBehaviorParam->Value(),
                   pBehaviorParam->GetText());
          BehaviorXMLParam behParam;

          behParam.setXMLTag(pBehaviorParam->Value());  // this should return a string value like
                                                        // "waypoint", "attitude_servo", etc.
          const char *str = pBehaviorParam->GetText();
          if (str != NULL) {
            behParam.setXMLTagValue(str);
          }

          TiXmlAttribute *pAttrib = pBehaviorParam->FirstAttribute();

          while (pAttrib != NULL) {
            ROS_INFO("\t\t\t Attirbute Name[%s], Value[%s]", pAttrib->Name(), pAttrib->Value());
            behParam.addXMLTagAttribute(pAttrib->Name(), pAttrib->Value());
            pAttrib = pAttrib->Next();
          }

          paramXmlTags.push_back(behParam);

          pBehaviorParam = pBehaviorParam->NextSiblingElement();
        }

        new_behavior->setBehaviorXmlParams(paramXmlTags);
        if (new_behavior->parseMissionFileParams() == false) {
          ROS_INFO("Error parsing mission file parameters. Skipping abort behavior");
        } else {
          // Add the abort node to the current mission
          mission.addAbortBehavior(new_behavior);
        }

        pABehavior = pABehavior->NextSiblingElement();
      }
    } else {
      ROS_WARN("Could not get abort behaviors");
      return false;
    }
  } catch (...)  // catch all exceptions
  {
    ROS_WARN("Parsing mission failed");
    return false;
  }

  return true;
}
/*
bool MissionParser::parseMissionFile(Mission& mission, const std::string& mission_file, const
std::string& schema_file)
{
        int rc;

        // Read the mission file
        xmlDocPtr doc = xmlReadFile(mission_file.c_str(), NULL, XML_PARSE_NONET);
        if (doc == NULL) {
                ROS_ERROR("%s(): Could not read mission file.", __FUNCTION__);
                return false;
        }

        // Make sure the mission is valid against the current mission schema
        rc = validateMissionDoc(doc, schema_file);
        if (rc != 0) return false;

        // Get the root (mission) node
        xmlNodePtr mission_node = xmlDocGetRootElement(doc);
        xmlNodePtr behaviors_node = findNodeChildElement(mission_node, "behaviors");
        xmlNodePtr abort_node = findNodeChildElement(mission_node, "abort");

        // Process mission runtime behavior nodes
        for (xmlNodePtr behavior_node = xmlFirstElementChild(behaviors_node); behavior_node;
behavior_node = behavior_node->next) { if (!strcmp((const char *)behavior_node->name,
"description")) continue; // skip past description element if (xmlNodeIsText(behavior_node))
continue; // skip the text nodes in behavior elements

                Behavior *new_behavior = m_factory->createBehavior((const char
*)behavior_node->name); if (new_behavior == NULL) { XML_WARN(behavior_node, "Behavior node does not
have an object factory [%s].", behavior_node->name); return false;
                }

                if (!new_behavior->getParams(node_handle)) {
                        ROS_ERROR("Failed to get configuration parameters for behavior [%s].",
behavior_node->name); return false;
                }

                // Route the parsing to the behavior library
                if (!new_behavior->parseXml(behavior_node)) {
                        ROS_ERROR("Failed to parse behavior node [%s].", behavior_node->name);
                        XML_WARN(behavior_node, "Failed to parse behavior node [%s].",
behavior_node->name); return false;
                }

                // Add the node to the current mission
                mission.addBehavior(new_behavior);

        }

        // Process the mission abort behavior nodes
        for (xmlNodePtr behavior_node = xmlFirstElementChild(abort_node); behavior_node;
behavior_node = behavior_node->next) { if (!strcmp((const char *)behavior_node->name,
"description")) continue; // skip past description element if (xmlNodeIsText(behavior_node))
continue; // skip the text nodes

                Behavior *new_behavior = m_factory->createBehavior((const char
*)behavior_node->name); if (new_behavior == NULL) { XML_WARN(behavior_node, "Behavior node does not
have an object factory [%s].", behavior_node->name); return false;
                }

                // Route the parsing to the behavior library
                if (!new_behavior->parseXml(behavior_node)) {
                        XML_WARN(behavior_node, "Failed to parse behavior node [%s].",
behavior_node->name); return false;
                }

                // Add the abort node to the current mission
                mission.addAbortBehavior(new_behavior);
        }

        // Clean up
        xmlFreeDoc(doc);
        xmlCleanupParser();

        return true;
}
*/

void MissionParser::cleanupMission(Mission &mission) {
  //	Behavior *behavior = NULL;

  //	while ((behavior = mission.getNextBehavior()) != NULL) {
  //		m_factory.destroyBehavior(behavior->getXmlTag(), behavior);
  //	}
}

/*
xmlNodePtr MissionParser::findNodeChildElement(xmlNodePtr parent, const char *name)
{
        for (xmlNodePtr child = xmlFirstElementChild(parent); child; child = child->next) {
                if (!strcmp((const char *)child->name, name)) return child;
        }

        return NULL;
}
*/
